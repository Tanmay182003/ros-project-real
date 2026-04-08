#!/usr/bin/env python3

import os
import threading
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
import azure.cognitiveservices.speech as speechsdk
from pynput import keyboard

AZURE_KEY = "591bb98703dc4187a6737380e4178155"
AZURE_REGION = "eastus"

# Command → (vel_left, vel_right)
COMMANDS = {
    "forward":  ( 0.5,  0.5),
    "go":       ( 0.5,  0.5),
    "straight": ( 0.5,  0.5),
    "left":     (-0.4,  0.4),
    "right":    ( 0.4, -0.4),
    "backward": (-0.3, -0.3),
    "back":     (-0.3, -0.3),
    "reverse":  (-0.3, -0.3),
    "stop":     ( 0.0,  0.0),
    "halt":     ( 0.0,  0.0),
}

PUBLISH_RATE = 10  # Hz


class VoiceControlNode(DTROS):

    def __init__(self, node_name):
        super(VoiceControlNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Current velocity state (protected by lock)
        self._lock = threading.Lock()
        self._vel_left = 0.0
        self._vel_right = 0.0

        # Push-to-talk state
        self._listening = False
        self._space_held = False

        # Azure Speech config
        speech_config = speechsdk.SpeechConfig(subscription=AZURE_KEY, region=AZURE_REGION)
        speech_config.speech_recognition_language = "en-US"
        audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
        self._recognizer = speechsdk.SpeechRecognizer(
            speech_config=speech_config,
            audio_config=audio_config
        )

        # Wire up callbacks
        self._recognizer.recognized.connect(self._on_recognized)
        self._recognizer.canceled.connect(self._on_canceled)

    def _set_velocity(self, vel_left, vel_right):
        with self._lock:
            self._vel_left = vel_left
            self._vel_right = vel_right

    def _get_velocity(self):
        with self._lock:
            return self._vel_left, self._vel_right

    def _on_recognized(self, evt):
        text = evt.result.text.strip().lower().rstrip(".")
        if not text:
            return
        rospy.loginfo(f"Heard: '{text}'")
        words = text.split()
        for keyword, (vl, vr) in COMMANDS.items():
            if keyword in words:
                rospy.loginfo(f"Command: {keyword} -> vel_left={vl}, vel_right={vr}")
                self._set_velocity(vl, vr)
                return
        rospy.logwarn(f"Unknown command: '{text}'")

    def _on_canceled(self, evt):
        rospy.logwarn(f"Speech recognition canceled: {evt.result.cancellation_details}")

    def _start_listening(self):
        if not self._listening:
            self._listening = True
            self._recognizer.start_continuous_recognition()
            rospy.loginfo("🎤 Listening... speak a command.")

    def _stop_listening(self):
        if self._listening:
            self._listening = False
            self._recognizer.stop_continuous_recognition()
            rospy.loginfo("🎤 Stopped listening.")

    def _on_key_press(self, key):
        if key == keyboard.Key.space and not self._space_held:
            self._space_held = True
            self._start_listening()

    def _on_key_release(self, key):
        if key == keyboard.Key.space:
            self._space_held = False
            self._stop_listening()

    def _publish_loop(self):
        """Continuously publish current velocity at PUBLISH_RATE Hz."""
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            vl, vr = self._get_velocity()
            msg = WheelsCmdStamped()
            msg.vel_left = vl
            msg.vel_right = vr
            self._publisher.publish(msg)
            rate.sleep()

    def run(self):
        rospy.loginfo("Voice control ready. Hold SPACEBAR to speak a command, release to stop listening.")

        # Start keyboard listener in its own thread
        listener = keyboard.Listener(
            on_press=self._on_key_press,
            on_release=self._on_key_release
        )
        listener.start()

        # Start continuous velocity publisher in a background thread
        pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        pub_thread.start()

        rospy.spin()

    def on_shutdown(self):
        rospy.loginfo("Stopping speech recognition and wheels...")
        if hasattr(self, '_recognizer') and self._listening:
            self._recognizer.stop_continuous_recognition()
        # Send stop command
        self._set_velocity(0.0, 0.0)
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)


if __name__ == '__main__':
    node = VoiceControlNode(node_name='voice_control_node')
    node.run()
