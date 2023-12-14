import io
import json
from typing import Iterator
import pyaudio
import wave
import sounddevice
import random
import string
import os
import openai
from google.cloud.speech_v2 import SpeechClient
from google.cloud.speech_v2.types import cloud_speech
from google.cloud import texttospeech
from rclpy.publisher import Publisher
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class ConversationPub(Node):
    def __init__(self):
        super().__init__("conversation_pub")
        self.publisher_ = self.create_publisher(String, '/keyword_topic', 10)
        self.publisher_end = self.create_publisher(String, 'conversation_end', 10)


    def publisher_keyword(self, keyword):
        msg = String()
        msg.data = keyword
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

class Conversation:
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    RECORD_SECONDS = 5
    project_id='gpt-hri-stt-and-tts'

    def __init__(self):
        self.client = SpeechClient()
        openai.api_key_path = '/home/geuny/robot_ws/openapi_key.txt'
        self.publisher = ConversationPub()

    def record(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)

        print("녹음을 시작합니다...")
        frames = []

        for i in range(0, int(self.RATE / self.CHUNK * self.RECORD_SECONDS)):
            data = stream.read(self.CHUNK)
            frames.append(data)

        print("녹음이 완료되었습니다.")

        stream.stop_stream()
        stream.close()
        p.terminate()

        filename = ''.join(random.choices(string.ascii_lowercase + string.digits, k=5)) + ".wav"
        wf = wave.open(filename, "wb")
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(p.get_sample_size(self.FORMAT))
        wf.setframerate(self.RATE)
        wf.writeframes(b"".join(frames))
        wf.close()

        print("파일이 " + filename + "로 저장되었습니다.")
        return filename

    def transcribe_file_v2(self, audio_file):
        recognizer_id = ''.join(random.choices(string.ascii_lowercase , k=5))

        request = cloud_speech.CreateRecognizerRequest(
            parent=f"projects/{self.project_id}/locations/global",
            recognizer_id=recognizer_id,
            recognizer=cloud_speech.Recognizer(
                language_codes=["ko-KR"], model="latest_long"
            ),
        )

        operation = self.client.create_recognizer(request=request)
        recognizer = operation.result()

        with io.open(audio_file, "rb") as f:        
            content = f.read()

        config = cloud_speech.RecognitionConfig(auto_decoding_config={})

        request = cloud_speech.RecognizeRequest(
            recognizer=recognizer.name, config=config, content=content
        )

        response = self.client.recognize(request=request)
        data = []
        for result in response.results:
            if result.alternatives:
                data.append(result.alternatives[0].transcript)

        print("인식 결과:")
        for transcript in data:
            print(transcript)

        os.remove(audio_file)

        return data

    def generate_response(self, prompt):
        response = openai.Completion.create(
            engine="text-davinci-003",
            prompt=prompt,
            max_tokens=50,
            temperature=0.7,
            n=1,
            stop=None
        )
        return response.choices[0].text.strip()

    def get_reply(self, prompt):
        model_name = "gpt-3.5-turbo"

        with open('message.json', 'r') as f:
            messages = json.load(f)
        messages.append({"role": "user", "content":prompt})

        response = openai.ChatCompletion.create(
          model=model_name,
          messages=messages
        )

        reply = response.choices[0].message['content']
        keywords = ['신발', '의류', '도서', '전자기기', '종료']
        for keyword in keywords:
            if keyword in reply:
                self.publisher.publisher_keyword(keyword)

        return reply

    def text_to_speech(self, text, filename):
        client = texttospeech.TextToSpeechClient()
        synthesis_input = texttospeech.SynthesisInput(text=str(text))
        voice = texttospeech.VoiceSelectionParams(
            language_code="ko-KR", name='ko-KR-Standard-D',ssml_gender=texttospeech.SsmlVoiceGender.MALE
        )
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.MP3
        )

        response = client.synthesize_speech(
            input=synthesis_input, voice=voice, audio_config=audio_config
        )

        with open(filename, "wb") as out:
            out.write(response.audio_content)
            print(f"Audio content written to file {filename}")
        return filename

    def play_audio(self, filename):
        os.system(f"mpg321 {filename}")

    def start_conversation(self):
        while True:
            audio_file = self.record()
            response = self.transcribe_file_v2(audio_file)

            if response is not None:
                for result in response:
                    prompt = result
                    if "그만" in prompt.lower():
                        print("대화를 종료합니다.")
                        exit_msg = String()
                        exit_msg.data = 'end'
                        exit_audio = "exit_msg.mp3"
                        self.text_to_speech(exit_audio)
                        self.play_audio(exit_audio)
                        return
                    reply = self.get_reply(prompt)
                    print("AI의 응답:")
                    print(reply)
                    reply_msg = String()
                    reply_msg.data = reply
                    self.publisher_.publish(reply_msg)
                    audio_filename = "response.mp3"
                    self.text_to_speech(reply, audio_filename)
                    self.play_audio(audio_filename)
            else:
                print("오디오 파일에서 텍스트 변환이 제대로 이루어지지 않았습니다.")

        rclpy.spin(self.publisher)   
        self.publisher.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    conversation = Conversation()
    conversation.start_conversation()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
