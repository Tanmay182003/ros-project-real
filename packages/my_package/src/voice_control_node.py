#!/usr/bin/env python3

import os
import threading
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
import azure.cognitiveservices.speech as speechsdk

AZURE_KEY = "591bb98703dc4187a6737380e4178155"
AZURE_REGION = "eastus"

# Command → (vel_left, vel_right)
COMMANDS = {
    "go":       ( 0.2,  0.2),
    "back":     (-0.2, -0.2),
    "stop":     ( 0.0,  0.0),
}

# Turn commands: (vel_left, vel_right, duration) during turn, then burst forward
TURN_COMMANDS = {
    "left":   (-0.3,  0.3,  0.5),
    "right":  ( 0.3, -0.3,  0.5),
}

# Slight turn commands: (vel_left, vel_right, duration) — adjust only, then stop
SLIGHT_COMMANDS = {
    "sloth":  (-0.1,  0.1, 0.2),
    "brick":  ( 0.1, -0.1, 0.2),
}

ALL_KEYWORDS = list(COMMANDS.keys()) + list(TURN_COMMANDS.keys()) + list(SLIGHT_COMMANDS.keys())

BURST_SPEED = 0.4   # fast burst speed
BURST_DURATION = 2.5 # seconds to go before auto-stopping

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

        # Track last acted-on command to avoid duplicates
        self._last_partial_cmd = None

        # Azure Speech config — tuned for lowest latency
        speech_config = speechsdk.SpeechConfig(subscription=AZURE_KEY, region=AZURE_REGION)
        speech_config.speech_recognition_language = "en-US"
        speech_config.set_property(speechsdk.PropertyId.SpeechServiceConnection_InitialSilenceTimeoutMs, "2000")
        speech_config.set_property(speechsdk.PropertyId.SpeechServiceConnection_EndSilenceTimeoutMs, "100")
        speech_config.set_property(speechsdk.PropertyId.Speech_SegmentationSilenceTimeoutMs, "100")
        speech_config.set_property(speechsdk.PropertyId.SpeechServiceResponse_StablePartialResultThreshold, "1")
        speech_config.set_profanity(speechsdk.ProfanityOption.Raw)
        speech_config.output_format = speechsdk.OutputFormat.Simple

        audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
        self._recognizer = speechsdk.SpeechRecognizer(
            speech_config=speech_config,
            audio_config=audio_config
        )

        # Boost recognition of our command keywords
        phrase_list = speechsdk.PhraseListGrammar.from_recognizer(self._recognizer)
        for word in ALL_KEYWORDS:
            phrase_list.addPhrase(word)

        # Wire up callbacks
        self._recognizer.recognizing.connect(self._on_recognizing)
        self._recognizer.recognized.connect(self._on_recognized)
        self._recognizer.canceled.connect(self._on_canceled)
        self._recognizer.session_started.connect(lambda evt: rospy.loginfo("SESSION: started"))
        self._recognizer.session_stopped.connect(lambda evt: rospy.logwarn("SESSION: stopped"))

    def _set_velocity(self, vel_left, vel_right):
        with self._lock:
            self._vel_left = vel_left
            self._vel_right = vel_right

    def _get_velocity(self):
        with self._lock:
            return self._vel_left, self._vel_right

    def _handle_text(self, text):
        text = text.strip().lower().rstrip(".")
        if not text:
            return
        words = text.split()
        # Check slight adjustments first
        for keyword, (vl, vr, dur) in SLIGHT_COMMANDS.items():
            if keyword in words and self._last_partial_cmd != keyword:
                self._last_partial_cmd = keyword
                threading.Thread(target=self._slight_adjust, args=(keyword, vl, vr, dur), daemon=True).start()
                return
        # Check turn commands
        for keyword, (vl, vr, dur) in TURN_COMMANDS.items():
            if keyword in words and self._last_partial_cmd != keyword:
                self._last_partial_cmd = keyword
                threading.Thread(target=self._turn, args=(keyword, vl, vr, dur), daemon=True).start()
                return
        for keyword, (vl, vr) in COMMANDS.items():
            if keyword in words and self._last_partial_cmd != keyword:
                self._last_partial_cmd = keyword
                if keyword in ("forward", "go", "straight"):
                    threading.Thread(target=self._burst_forward, daemon=True).start()
                elif keyword in ("backward", "back", "reverse"):
                    threading.Thread(target=self._burst_backward, daemon=True).start()
                else:
                    rospy.loginfo(f"Command: {keyword} -> vel_left={vl}, vel_right={vr}")
                    self._set_velocity(vl, vr)
                return

    def _burst_forward(self):
        rospy.loginfo(f"BURST forward for {BURST_DURATION}s")
        self._set_velocity(BURST_SPEED, BURST_SPEED)
        rospy.sleep(BURST_DURATION)
        rospy.loginfo("BURST complete, stopping.")
        self._set_velocity(0.0, 0.0)
        self._last_partial_cmd = None

    def _burst_backward(self):
        rospy.loginfo(f"BURST backward for {BURST_DURATION}s")
        self._set_velocity(-BURST_SPEED, -BURST_SPEED)
        rospy.sleep(BURST_DURATION)
        rospy.loginfo("BURST complete, stopping.")
        self._set_velocity(0.0, 0.0)
        self._last_partial_cmd = None

    def _turn(self, keyword, vl, vr, duration):
        rospy.loginfo(f"Command: {keyword} -> turning for {duration}s")
        self._set_velocity(vl, vr)
        rospy.sleep(duration)
        self._set_velocity(vl, vr)

    def _slight_adjust(self, keyword, vl, vr, duration):
        rospy.loginfo(f"Command: {keyword} -> slight adjust for {duration}s")
        self._set_velocity(vl, vr)
        rospy.sleep(duration)
        self._set_velocity(0.0, 0.0)
        self._last_partial_cmd = None

    def _on_recognizing(self, evt):
        self._handle_text(evt.result.text)

    def _on_recognized(self, evt):
        if evt.result.reason == speechsdk.ResultReason.RecognizedSpeech:
            text = evt.result.text.strip().lower().rstrip(".")
            if text:
                rospy.loginfo(f"FINAL: '{text}'")
                self._last_partial_cmd = None
        elif evt.result.reason == speechsdk.ResultReason.NoMatch:
            rospy.logwarn(f"NO MATCH: {evt.result.no_match_details.reason}")

    def _on_canceled(self, evt):
        details = evt.result.cancellation_details
        rospy.logwarn(f"CANCELED: reason={details.reason}")
        if details.reason == speechsdk.CancellationReason.Error:
            rospy.logerr(f"ERROR: code={details.error_code}, details={details.error_details}")

    def _publish_loop(self):
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            vl, vr = self._get_velocity()
            msg = WheelsCmdStamped()
            msg.vel_left = vl
            msg.vel_right = vr
            self._publisher.publish(msg)
            rate.sleep()

    def run(self):
        rospy.loginfo("Voice control ready. Say a command: go, left, right, back, stop, sloth, brick")

        self._recognizer.start_continuous_recognition()

        pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        pub_thread.start()

        rospy.spin()

    def on_shutdown(self):
        rospy.loginfo("Stopping speech recognition and wheels...")
        if hasattr(self, '_recognizer'):
            self._recognizer.stop_continuous_recognition()
        self._set_velocity(0.0, 0.0)
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)


if __name__ == '__main__':
    node = VoiceControlNode(node_name='voice_control_node')
    node.run()
