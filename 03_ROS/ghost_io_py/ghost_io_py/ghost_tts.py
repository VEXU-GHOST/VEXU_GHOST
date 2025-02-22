#!/usr/bin/env python3
# thanks chat, https://chatgpt.com/share/67b95b69-6e78-800c-a0a8-64583e95fa80
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tempfile
import wave
import subprocess
import os
import random
from pathlib import Path
from typing import Any, Dict, Optional, List

# Import PiperVoice and auto-download features from the Piper TTS library.
from piper import PiperVoice
from piper.download import ensure_voice_exists, find_voice, get_voices

class TTSMusicNode(Node):
    def __init__(self) -> None:
        super().__init__('tts_music_node')
        self.get_logger().info("Initializing TTSMusicNode with auto-download support")

        # Declare parameters for PiperVoice and auto-download.
        # Default model is set to "en_GB-alan-low"
        self.declare_parameter("model", "en_US-ryan-low")
        self.declare_parameter("config", "")
        self.declare_parameter("data_dir", [str(Path.cwd())])
        self.declare_parameter("download_dir", "")
        self.declare_parameter("update_voices", False)
        self.declare_parameter("use_cuda", False)
        self.declare_parameter("music_folder", "~/VEXU_GHOST/sound_effects")

        model: str = self.get_parameter("model").value
        config: str = self.get_parameter("config").value
        data_dir: List[str] = self.get_parameter("data_dir").value
        download_dir: str = self.get_parameter("download_dir").value
        update_voices: bool = self.get_parameter("update_voices").value
        use_cuda: bool = self.get_parameter("use_cuda").value

        if not download_dir:
            download_dir = data_dir[0]
            self.get_logger().info(f"No download_dir provided, defaulting to {download_dir}")

        model_path: Path = Path(model)
        if not model_path.exists():
            self.get_logger().info(f"Model '{model}' not found locally. Initiating auto-download.")
            try:
                voices_info: Dict[str, Any] = get_voices(download_dir, update_voices=update_voices)
                aliases_info: Dict[str, Any] = {}
                for voice_info in voices_info.values():
                    for voice_alias in voice_info.get("aliases", []):
                        aliases_info[voice_alias] = {"_is_alias": True, **voice_info}
                voices_info.update(aliases_info)
                ensure_voice_exists(model, data_dir, download_dir, voices_info)
                model, config = find_voice(model, data_dir)
                self.get_logger().info(f"Model downloaded and resolved: {model}")
            except Exception as e:
                model, config = find_voice(model, data_dir)
                self.get_logger().error(f"Auto-download failed: {e}")
                #raise e
        else:
            self.get_logger().info(f"Using local model: {model}")

        try:
            self.voice: PiperVoice = PiperVoice.load(model, config_path=config, use_cuda=use_cuda)
            self.get_logger().info("PiperVoice loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load PiperVoice: {e}")
            raise e

        # Define synthesis arguments.
        # Removed "speaker_id" as it may cause invalid input error with certain models.
        self.synthesize_args: Dict[str, Any] = {
            "length_scale": 1.0,
            "noise_scale": 0.667,
            "noise_w": 0.8,
            "sentence_silence": 0.5,
        }
        #pactl set-sink-volume @DEFAULT_SINK@ 100%
        subprocess.run(["pactl", "set-sink-volume", "@DEFAULT_SINK@", "100%"], capture_output=True, text  = True)

        # Create subscriptions for TTS and music topics.
        self.create_subscription(String, "/io/speaker/tts", self.tts_callback, 10)
        self.create_subscription(String, "/io/speaker/music", self.music_callback, 10)

    def tts_callback(self, msg: String) -> None:
        """
        Callback for TTS messages.
        Synthesizes speech from the received text and plays the resulting WAV file.
        """
        text: str = msg.data.strip()
        if not text:
            self.get_logger().warn("Received empty TTS message; ignoring.")
            return

        self.get_logger().info(f"Received TTS message: '{text}'")
        try:
            # Create a temporary WAV file for synthesized audio.
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_file:
                tmp_filename: str = tmp_file.name

            with wave.open(tmp_filename, "wb") as wav_file:
                self.voice.synthesize(text, wav_file, **self.synthesize_args)

            # Play the synthesized audio using an external audio player.
            subprocess.run(["play", tmp_filename], capture_output=True, text  = True)
        except Exception as e:
            self.get_logger().warn(f"Error during TTS synthesis: {e}")

    def music_callback(self, msg: String) -> None:
        """
        Callback for music messages.
        Plays a music file from a designated folder. If the provided key matches part
        of a filename, that file is played. Otherwise, a random file from the folder is chosen.
        Before attempting to play, the file's existence is verified.
        """
        key: str = msg.data.strip().lower()
        music_folder: str = os.path.expanduser(self.get_parameter("music_folder").value)

        if not os.path.isdir(music_folder):
            self.get_logger().warn(f"Music folder '{music_folder}' does not exist or is not a directory.")
            return

        # List all .wav files in the music folder
        music_files: List[str] = [
            os.path.join(music_folder, f)
            for f in os.listdir(music_folder)
            if f.lower().endswith(".wav") or f.lower().endswith("mp3")
        ]

        if not music_files:
            self.get_logger().warn(f"No .wav files found in folder '{music_folder}'.")
            return

        # Find all files whose name contains the key
        matching_files: List[str] = [
            file for file in music_files if key in os.path.basename(file).lower()
        ]

        if len(matching_files) == 1:
            selected_file: str = matching_files[0]
        else:
            if matching_files:
                self.get_logger().warn(f"Multiple music files match key '{key}'; selecting a random one from matches.")
                selected_file = random.choice(matching_files)
            else:
                self.get_logger().warn(f"Music key '{key}' not found; selecting a random file from all files.")
                selected_file = random.choice(music_files)


        if not os.path.isfile(selected_file):
            self.get_logger().warn(f"Selected music file '{selected_file}' does not exist.")
            return

        self.get_logger().info(f"Playing music file: {selected_file}")
        try:
            subprocess.run(["play", selected_file], capture_output=True, text  = True)
        except Exception as e:
            self.get_logger().warn(f"Error playing music file '{selected_file}': {e}")


def main(args: Optional[Any] = None) -> None:
    rclpy.init(args=args)
    node: TTSMusicNode = TTSMusicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TTSMusicNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
