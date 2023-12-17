import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import io
import json
import pyaudio
import wave
import tempfile
import sounddevice
import random
import string
import os
import openai
import time
from google.cloud import speech
from google.cloud import texttospeech
from rclpy.publisher import Publisher


class ConversationStateMachine(Node):

    def __init__(self):
        super().__init__('conversation_state_machine')
        self.publisher_ = self.create_publisher(String, '/keyword_topic', 10)
        self.current_state = "WAITING"
        self.intro_msg = "안녕하세요. 저는 백화점 안내로봇 라이언입니다. 도움이 필요하신가요?"

        # Initialize Google Cloud client
        self.client = speech.SpeechClient()
        # Set up pyaudio
        self.p = pyaudio.PyAudio()
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.RECORD_SECONDS = 5

        # Set up OpenAI API key
        openai.api_key_path = '/home/geuny/robot_ws/openapi_key.txt'
        self.speech_client = speech.SpeechClient()
        self.request_text = ""
        self.arrival_flag = 0 


    def run_state_machine(self):
        if self.current_state == "WAITING":
            # 손님 감지 로직 (여기서는 예시를 위해 자동으로 GREETING 상태로 전환)
            self.transition_to_state("GREETING")

        elif self.current_state == "GREETING":
            self.speak_response(self.intro_msg)  # 인사 메시지를 음성으로 전달
            self.transition_to_state("CONFIRMING_SERIVCE_USE")

        elif self.current_state == "CONFIRMING_SERVICE_USE":
            self.record_audio()  # 음성 녹음 시작
            self.request_text = self.transcribe_audio("path_to_recorded_audio.wav")  # 음성을 텍스트로 변환
            if "네" in self.request_text or "응" in self.request_text:
                self.transition_to_state("EXPLAINING_USAGE")
            else:
                self.transition_to_state("GENERAL_CONVERSATION")

        elif self.current_state == "EXPLAINING_USAGE":
            usage_message = "손님에게 도움을 드릴 수 있어 기쁩니다..."
            self.publisher_.publish(String(data="[Gesture][Explain]")) # 설명할 때의 제스처 발행
            self.speak_response(usage_message)  # 사용 방법 설명
            self.transition_to_state("LISTENING_FOR_REQUEST")

        elif self.current_state == "LISTENING_FOR_REQUEST":
            self.publisher_.publish(String(data="[Gesture][Recording]"))  # 요청 듣기 전 제스처 발행
            self.record_audio()  # 음성 녹음 시작
            self.request_text = self.transcribe_audio("path_to_recorded_audio.wav")  # 음성을 텍스트로 변환

            if self.arrival_flag > 0:
                if "종료" in self.request_text:
                    self.transition_to_state("END_CONFIRM")
                elif any(keyword in self.request_text for keyword in ["옷", "신발", "전자기기", "책"]):
                    self.transition_to_state("EXPLAINING_USAGE")
                elif "얼마야" in self.request_text or "구매" in self.request_text:
                    self.transition_to_state("SALES")
                else:
                    self.transition_to_state("GENERAL_CONVERSATION")
            else:
                self.transition_to_state("KEYWORD_ANALYSIS")


        elif self.current_state == "PROCESSING_REQUEST":
            self.publisher_.publish(String(data="[Gesture][ProcessingText]"))  # 텍스트 처리 전 제스처 발행
            response = self.generate_response(self.request_text)  # 요청에 대한 응답 생성
            self.speak_response(response)  # 응답을 음성으로 전달
            # 추가 로직에 따라 상태 전환 결정 (예: 다시 LISTENING_FOR_REQUEST 또는 ENDING)
        
        elif self.current_state == "KEYWORD_ANALYSIS":
            if any(keyword in self.request_text for keyword in ["옷", "신발", "전자기기", "책"]):
                self.transition_to_state("PROVIDING_INFORMATION")
            else:
                self.transition_to_state("GENERAL_CONVERSATION")

        elif self.current_state == "PROVIDING_INFORMATION":
            keyword = self.identify_keyword(self.request_text)
            explanation = self.get_keyword_explanation(keyword)
            self.speak_response(explanation)
            self.transition_to_state("CONFIRMING_MOVEMENT")

        elif self.current_state == "CONFIRMING_MOVEMENT":
            keyword = self.identify_keyword(self.request_text)  # 키워드 식별
            self.speak_response(f"{keyword} 코너로 안내해 드릴까요?")
            self.record_audio()
            response_text = self.transcribe_audio("path_to_recorded_audio.wav")

            if "네" in response_text or "응" in response_text:
                # 사용자가 이동을 원할 경우
                self.publisher_.publish(String(data=f"[Navigation][Guide_{keyword}]"))  # 네비게이션 팀에게 안내 요청
            else:
                # 사용자가 이동을 원하지 않을 경우
                self.speak_response("추가적으로 도와드릴 부분이 있을까요?")
                self.transition_to_state("GENERAL_CONVERSATION")
        
        elif self.current_state == "MOVING":
            self.get_logger().info("이동 중입니다...")
            time.sleep(10)  # 10초 대기를 시뮬레이션
            self.transition_to_state("ARRIVAL")

        elif self.current_state == "ARRIVAL":
            arrival_message = f"{keyword} 코너에 도착하였습니다. ..."
            self.speak_response(arrival_message)
            self.arrival_flag += 1
            self.transition_to_state("LISTENING_FOR_REQUEST")
        
        elif self.current_state == "SALES":
            # "[HRI][<keyword>_sales]" 토픽 메시지 발행
            self.publisher_.publish(String(data=f"[HRI][{self.current_keyword}_sales]"))
            
            # .txt 파일에서 긴 응답 메시지를 불러와 TTS로 변환하고 재생
            self.speak_long_response(f"{self.current_keyword}_sales_message.txt")

            # 구매 결정 확인을 위해 PURCHASE_CONFIRM 상태로 전환
            self.transition_to_state("PURCHASE_CONFIRM")



        elif self.current_state == "PURCHASE_CONFIRM":
            confirm_purchase_msg = "이 상품이 마음에 드시나요? ..."
            self.speak_response(confirm_purchase_msg)
            self.record_audio()
            response_text = self.transcribe_audio("path_to_recorded_audio.wav")

            if "네" in response_text or "응" in response_text:
                self.speak_response("구매를 결정해주셔서 감사합니다.")
                self.publisher_.publish(String(data=f"[HRI][{keyword}_purchase]"))
                self.speak_response("더 도와드릴 부분이 있을까요?")
                self.transition_to_state("LISTENING_FOR_REQUEST")
            else:
                self.speak_response("다른 상품을 둘러보시겠어요?")
                self.transition_to_state("LISTENING_FOR_REQUEST")

        elif self.current_state == "END_CONFIRM":
            confirm_end_msg = "정말로 서비스를 종료하시겠어요?"
            self.speak_response(confirm_end_msg)
            self.record_audio()
            response_text = self.transcribe_audio("path_to_recorded_audio.wav")

            if "네" in response_text or "응" in response_text:
                self.transition_to_state("ENDING")
            else:
                self.speak_response("더 도와드릴 것이 있나요?")
                self.transition_to_state("LISTENING_FOR_REQUEST")

        elif self.current_state == "GENERAL_CONVERSATION":
            # 일반 대화 로직 처리
            response = self.generate_response(self.request_text)
            self.speak_response(response)
            self.transition_to_state("LISTENING_FOR_REQUEST")

        elif self.current_state == "ENDING":
            goodbye_message = "감사합니다. 즐거운 쇼핑 되십시오."
            self.speak_response(goodbye_message)  # 종료 인사
            self.publisher_.publish(String(data="[Navigation][Return]"))  # 종료 및 복귀 제스처 발행
            # 필요한 종료 처리 수행

    def transition_to_state(self, new_state):
        if new_state == "GREETING" and self.current_state == "WAITING":
            self.get_logger().info(f'손님이 감지되었습니다. {new_state} 상태로 전환합니다.')
            self.current_state = new_state
            # Trigger the greeting action
            self.greet_customer()
        else:
            self.get_logger().error(f'Invalid state transition from {self.current_state} to {new_state}')

        if new_state == "EXPLAINING_USAGE" and self.current_state == "CONFIRMING_SERIVCE_USE":
            self.get_logger().info(f'사용 방법 설명을 시작합니다.')
            self.current_state = new_state
        elif new_state == "ENDING" and (self.current_state == "CONFIRMING_SERIVCE_USE" or self.current_state == "EXPLAINING_USAGE"):
            self.get_logger().info(f'대화를 마치고 로봇을 반환합니다.')
            self.current_state = new_state

    def greet_customer(self):
        # This function will use text-to-speech to greet the customer
        self.get_logger().info(f'인사 메시지를 준비합니다: "{self.intro_msg}"')
        # Here we would call the text-to-speech service to generate and play the greeting message
        # For demonstration, we'll just log the message and publish it
        self.publisher_.publish(String(data=self.intro_msg))
        self.get_logger().info(f'인사 메시지를 발행하였습니다: "{self.intro_msg}"')
        # After greeting, we might want to transition to another state, such as listening for a response
        # self.transition_to_state("CONFIRMING_SERIVCE_USE")
        
    # 기존에 정의된 라이브러리와 ConversationStateMachine 클래스에 이어서...

    def listen_to_customer_response(self):
        # This function will simulate listening to the customer response.
        # Here, we assume the customer always says "yes" for demonstration.
        # In a real scenario, this would be handled by an actual STT service.
        customer_response = "네"  # Or "아니오", depending on the actual customer response
        self.get_logger().info(f'손님의 대답: "{customer_response}"')
        if customer_response == "네":
            self.transition_to_state("EXPLAINING_USAGE")
        elif customer_response == "아니오":
            self.transition_to_state("ENDING")

    def explaining_usage(self):
        # This function explains how to interact with the robot using gestures
        usage_msg = "손님에게 도움을 드릴 수 있어 기쁩니다. 제가 이렇게 양 팔을 흔들고 있으면 손님의 말씀을 잘 듣는 중인거예요. 그리고 제가 이렇게 한 손을 번쩍 들고 있으면, 손님을 도와드리기 위해 곰곰이 생각하는 중인겁니다! 지금부터 안내를 시작하겠습니다. 무엇을 도와드릴까요?"
        self.publisher_.publish(String(data="[Gesture][Recording]"))  # 로봇이 듣는 중임을 알리는 제스처 발행
        self.get_logger().info(f'사용 방법 설명: "{usage_msg}"')
        # Here we would call the text-to-speech service to speak out the usage message
        # Transition to the next state, for example LISTENING_FOR_REQUEST

    def ending_interaction(self):
        goodbye_msg = "감사합니다. 즐거운 쇼핑 되십시오."
        self.publisher_.publish(String(data="[Navigation][Return]"))  # 로봇이 복귀하라는 명령 발행
        self.get_logger().info(f'끝인사: "{goodbye_msg}"')
        # Here we would call the text-to-speech service to speak out the goodbye message
        # No need to transition to another state, this could be the final state
    
    def listening(self):
        # 로봇이 사용자의 음성을 듣고 있다는 것을 알리는 제스처를 시작합니다.
        self.publisher_.publish(String(data="[Gesture][Recording]"))
        self.get_logger().info("음성 입력을 기다리는 중...")
        # 여기에서 음성 녹음을 시작합니다.
        audio_file = self.record_audio()  # 음성 녹음 메소드를 호출합니다.
        # 녹음된 음성을 텍스트로 변환합니다.
        transcript = self.transcribe_audio(audio_file)
        if "네" in transcript or "응" in transcript:
            self.transition_to_state("EXPLAINING_USAGE")
        else:
            self.transition_to_state("ENDING")

    def explaining_usage(self):
        # ... 이전에 정의된 explaining_usage 메서드 ...
        # 사용자에게 상태를 설명한 후, 사용자의 요청을 듣기 위해 다음 상태로 전환합니다.
        self.transition_to_state("LISTENING_FOR_REQUEST")

    def listening_for_request(self):
        # 사용자의 요청을 듣기 위한 상태입니다.
        self.publisher_.publish(String(data="[Gesture][Recording]"))
        self.get_logger().info("사용자의 요청을 듣는 중...")
        # 여기에서 음성 녹음을 시작합니다.
        audio_file = self.record_audio()  # 음성 녹음 메소드를 호출합니다.
        # 녹음된 음성을 텍스트로 변환합니다.
        request_text = self.transcribe_audio(audio_file)
        # 요청을 처리하는 다음 상태로 전환합니다.
        self.transition_to_state("PROCESSING_REQUEST")
        self.request_text = self.transcribe_audio(audio_file)
        self.processing_request()


    def processing_request(self):
        # ... 요청을 처리하고 응답을 생성하는 로직 ...
        self.publisher_.publish(String(data="[Gesture][ProcessingText]"))
        self.get_logger().info("요청을 처리하고 응답을 생성하는 중...")
        # 요청에 대한 응답을 생성합니다.
        response = self.generate_response(self.request_text)  # 텍스트 응답 생성 메소드를 호출합니다.
        # 응답을 음성으로 합성하고 재생합니다.
        self.speak_response(response)
        # 추가 대화가 필요한 경우 또는 대화를 종료할 경우 다음 상태로 전환합니다.
        self.transition_to_state("WAITING")  # 또는 "ENDING"
    
    
    def record_audio(self):
        # PyAudio 설정
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)

        print("녹음 시작")
        frames = []

        # 녹음 시작
        for _ in range(0, int(16000 / 1024 * 5)):  # 5초 동안 녹음
            data = stream.read(1024)
            frames.append(data)

        print("녹음 종료")
        
        # 녹음 종료 후 파일 저장
        stream.stop_stream()
        stream.close()
        p.terminate()

        # 임시 파일 생성 및 녹음된 오디오 데이터 저장
        filepath = tempfile.mktemp(suffix=".wav")
        with wave.open(filepath, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
            wf.setframerate(16000)
            wf.writeframes(b''.join(frames))

        return filepath


    def transcribe_audio(self, audio_file):
        # Google Cloud Speech-to-Text를 사용하여 오디오 파일을 텍스트로 변환
        with io.open(audio_file, 'rb') as audio_stream:
            content = audio_stream.read()

        audio = speech.RecognitionAudio(content=content)
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code="ko-KR"
        )

        response = self.speech_client.recognize(config=config, audio=audio)
        transcript = response.results[0].alternatives[0].transcript if response.results else ""

        return transcript
    

    def generate_response(self, request_text):
        # OpenAI GPT-3를 사용하여 텍스트 응답 생성
        response = openai.Completion.create(
            engine="text-davinci-003",
            prompt=request_text,
            max_tokens=50,
            temperature=0.7,
            n=1,
            stop=None
        )
        return response.choices[0].text.strip()

    def speak_response(self, response_text):
        # Google Cloud Text-to-Speech를 사용하여 텍스트를 음성으로 변환
        synthesis_input = texttospeech.SynthesisInput(text=response_text)
        voice = texttospeech.VoiceSelectionParams(
            language_code="ko-KR", ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL
        )
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.MP3
        )

        response = self.text_to_speech_client.synthesize_speech(
            input=synthesis_input, voice=voice, audio_config=audio_config
        )

        # 음성 파일 저장 및 재생
        audio_file = tempfile.mktemp(suffix='.mp3')
        with open(audio_file, 'wb') as out:
            out.write(response.audio_content)
        return audio_file
    
    def play_audio(self, audio_file):
        # Code for playing the synthesized audio
        os.system(f"mpg321 {audio_file}")

    def get_keyword_explanation(self, keyword):
        # 키워드에 해당하는 설명 파일을 읽어서 TTS로 전달
        filename = f"{keyword}_explain.txt"
        if os.path.exists(filename):
            with open(filename, 'r', encoding='utf-8') as file:
                return file.read()
        else:
            return "해당 키워드에 대한 설명이 없습니다."
    
    def speak_long_response(self, filename):
        """긴 응답을 .txt 파일에서 불러와 TTS로 변환하고 재생하는 메소드"""
        if os.path.exists(filename):
            with open(filename, 'r', encoding='utf-8') as file:
                long_response = file.read()
            self.speak_response(long_response)
        else:
            self.get_logger().error(f"파일 {filename}을 찾을 수 없습니다.")


def main(args=None):
    rclpy.init(args=args)
    state_machine = ConversationStateMachine()

    try:
        while rclpy.ok():
            rclpy.spin_once(state_machine)
            state_machine.run_state_machine()
    except KeyboardInterrupt:
        pass
    finally:
        state_machine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

