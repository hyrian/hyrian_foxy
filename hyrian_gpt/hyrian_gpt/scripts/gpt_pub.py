import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import io
import pyaudio
import sounddevice
import wave
import tempfile
import os
import openai
import time
from google.cloud import speech_v2 as speech
from google.cloud import speech
from google.cloud.speech_v2.types import cloud_speech
from google.cloud.speech import RecognitionAudio
from google.cloud import texttospeech
import logging
import threading
import pygame
import json
import sys


class ConversationStateMachine(Node):
    def __init__(self):
        super().__init__('conversation_state_machine')
        self.publisher_ = self.create_publisher(String, '/keyword_topic', 10)
        self.subscription = self.create_subscription(
        String, 
        '/keyword_topic', 
        self.listener_callback, 
        10)
        self.subscription  # prevent unused variable warning

        self.current_state = "WAITING"
        self.intro_msg = "안녕하세요. 저는 백화점 안내로봇 라이언입니다. 도움이 필요하신가요?"
        self.client = speech.SpeechClient()
        self.p = pyaudio.PyAudio()
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.speech_client = speech.SpeechClient()
        self.text_to_speech_client = texttospeech.TextToSpeechClient()
        self.CHANNELS = 1
        self.RATE = 16000
        self.RECORD_SECONDS = 5        
        openai.api_key_path = '/home/geuny/robot_ws/openapi_key.txt'
        self.request_text = ""
        self.arrival_flag = 0
        self.keyword = None
        self.base_path = "/home/geuny/robot_ws/src/gpt_test/text"
        project_id = 'gpt-hri-stt-and-tts'

    def run_state_machine(self):
        try:
            if self.current_state == "WAITING":
                self.publisher_.publish(String(data="State,Waiting"))
                time.sleep(5)
                self.transition_to_state("GREETING")

            elif self.current_state == "GREETING":
                self.publisher_.publish(String(data="Gesture,Greeting"))
                self.speak_response(self.intro_msg)
                self.transition_to_state("CONFIRMING_SERVICE_USE")

            elif self.current_state == "CONFIRMING_SERVICE_USE":
                self.publisher_.publish(String(data="State,ConfirmServiceUse"))
                audio_file_path = self.record_audio()  # record_audio 메소드에서 파일 경로 반환
                self.request_text = self.transcribe_file_v2(audio_file_path)  # 반환된 경로 사용
                if "네" in self.request_text or "응" in self.request_text:
                    self.transition_to_state("EXPLAINING_USAGE")
                elif "아니" in self.request_text:
                    self.transition_to_state("END_CONFIRM")
                else:
                    self.transition_to_state("RECONFIRMING_USE")

            elif self.current_state == "RECONFIRMING_USE":
                reconfirm_message = "죄송하지만, 먼저 저를 사용하실지를 결정해주시겠어요? 응 또는 아니로 대답을 부탁드립니다."
                self.publisher_.publish(String(data="State,Reconfirm"))
                self.speak_response(reconfirm_message)
                audio_file_path = self.record_audio()  # record_audio 메소드에서 파일 경로 반환
                self.request_text = self.transcribe_file_v2(audio_file_path)  # 반환된 경로 사용
                if "네" in self.request_text or "응" in self.request_text:
                    self.transition_to_state("EXPLAINING_USAGE")
                elif "아니" in self.request_text:
                    self.transition_to_state("END_CONFIRM")
                else:
                    self.current_state == "RECONFIRMING_USE"



            elif self.current_state == "EXPLAINING_USAGE":
                usage_message = "손님에게 도움을 드릴 수 있어 기쁩니다.제가 이렇게 양 팔을 흔들고 있으면 손님의 말씀을 잘 듣는 중인거예요. 그리고 제가 이렇게 한 손을 번쩍 들고 있으면, 손님을 도와드리기 위해 곰곰이 생각하는 중인겁니다! 서비스를 종료하고 싶으시다면 종료라고 말해주세요. 그리고 어떤 물건을 구매하고 싶으시다면 구매라고 말씀해주세요. 지금부터 안내를 시작하겠습니다. 무엇을 도와드릴까요?"
                self.publisher_.publish(String(data="Gesture,Explain"))
                self.speak_response(usage_message)
                self.transition_to_state("LISTENING_FOR_REQUEST")

            elif self.current_state == "LISTENING_FOR_REQUEST":
                self.publisher_.publish(String(data="Gesture,Recording"))
                audio_file_path = self.record_audio()  # record_audio 메소드에서 파일 경로 반환
                self.request_text = self.transcribe_file_v2(audio_file_path)  # 반환된 경로 사용

                if self.arrival_flag > 0:
                    keywords = ["옷", "신발", "전자기기", "책"]
                    # 현재 키워드를 목록에서 제외
                    filtered_keywords = [keyword for keyword in keywords if keyword != self.keyword]               
                    if any(keyword in self.request_text for keyword in filtered_keywords):
                        self.keyword = self.identify_keyword(self.request_text)
                        self.transition_to_state("PROVIDING_INFORMATION")
                    elif "얼마야" in self.request_text or "구매" in self.request_text:
                        self.transition_to_state("SALES")
                    elif "종료" in self.request_text:
                        self.transition_to_state("END_CONFIRM")
                    else:
                        self.transition_to_state("GENERAL_CONVERSATION")
                else:
                    if "종료" in self.request_text:
                        self.transition_to_state("END_CONFIRM")
                    else:
                        self.transition_to_state("KEYWORD_ANALYSIS")

            
            elif self.current_state == "PROCESSING_REQUEST":
                self.publisher_.publish(String(data="Gesture,ProcessingText"))
                response = self.get_reply(self.request_text)
                self.speak_response(response)
                # Additional logic to determine state transition (e.g., back to LISTENING_FOR_REQUEST or ENDING)

            elif self.current_state == "KEYWORD_ANALYSIS":
                self.publisher_.publish(String(data="State,KeywordAnalysis"))
                if any(keyword in self.request_text for keyword in ["옷", "신발", "전자기기", "책"]):
                    self.transition_to_state("PROVIDING_INFORMATION")
                else:
                    self.transition_to_state("GENERAL_CONVERSATION")

            elif self.current_state == "PROVIDING_INFORMATION":
                self.keyword = self.identify_keyword(self.request_text)  # keyword 업데이트
                self.publisher_.publish(String(data="State,ProvidingInformation"))
                explain_filename = f"{self.keyword}_explain.txt"
                self.speak_long_response(explain_filename)
                self.transition_to_state("CONFIRMING_MOVEMENT")

            elif self.current_state == "CONFIRMING_MOVEMENT":
                # self.keyword 사용 (이미 "PROVIDING_INFORMATION"에서 설정됨)
                self.publisher_.publish(String(data="State,ConfirmingMovement"))
                self.speak_response(f"{self.keyword} 코너로 안내해 드릴까요?")
                audio_file_path = self.record_audio()
                response_text = self.transcribe_file_v2(audio_file_path)

                if "네" in response_text or "응" in response_text:
                    self.publisher_.publish(String(data=f"Navigation,Guide_{self.keyword}"))
                    self.transition_to_state("MOVING")
                else:
                    self.speak_response("추가적으로 도와드릴 부분이 있을까요?")
                    self.transition_to_state("LISTENING_FOR_REQUEST")

            elif self.current_state == "MOVING":
                self.get_logger().info("이동 중입니다...")
                self.speak_response("안내를 시작하겠습니다.")
                self.publisher_.publish(String(data="State,Moving"))
                time.sleep(10)  # 10초 대기를 시뮬레이션
                self.transition_to_state("ARRIVAL")

            elif self.current_state == "ARRIVAL":
                # self.keyword 사용 (이미 "PROVIDING_INFORMATION"에서 설정됨)
                arrival_message_filename = f"{self.keyword}_arrival.txt"
                self.publisher_.publish(String(data="State,Arrival"))
                self.speak_long_response(arrival_message_filename)
                self.arrival_flag += 1
                self.transition_to_state("LISTENING_FOR_REQUEST")

            
            elif self.current_state == "SALES":
                # "[HRI][<keyword>_sales]" 토픽 메시지 발행
                self.publisher_.publish(String(data=f"HRI,{self.keyword}_sales"))
                
                # .txt 파일에서 긴 응답 메시지를 불러와 TTS로 변환하고 재생
                self.speak_long_response(f"{self.keyword}_sales_message.txt")

                # 구매 결정 확인을 위해 PURCHASE_CONFIRM 상태로 전환
                self.transition_to_state("PURCHASE_CONFIRM")

            elif self.current_state == "PURCHASE_CONFIRM":
                confirm_purchase_msg = "이 상품이 마음에 드시나요? 상품에 부착된 QR코드를 통해 결제를 진행하실 수 있습니다."

                self.speak_response(confirm_purchase_msg)
                    # 녹음 시작
                audio_file_path = self.record_audio()
    
                # 음성 인식 시작
                response_text = self.transcribe_file_v2(audio_file_path)
                if "네" in response_text or "응" in response_text:
                    self.speak_response("구매를 결정해주셔서 감사합니다.")
                    self.publisher_.publish(String(data=f"HRI,{self.keyword}_purchase"))
                    self.speak_response("더 도와드릴 부분이 있을까요?")
                    self.transition_to_state("LISTENING_FOR_REQUEST")
                else:
                    self.speak_response("다른 상품을 둘러보시겠어요?")
                    self.transition_to_state("LISTENING_FOR_REQUEST")

            elif self.current_state == "END_CONFIRM":
                confirm_end_msg = "정말로 서비스를 종료하시겠어요?"
                self.publisher_.publish(String(data="State,EndConfirm"))
                self.speak_response(confirm_end_msg)
                audio_file_path = self.record_audio()
                response_text = self.transcribe_file_v2(audio_file_path)

                if "네" in response_text or "응" in response_text:
                    self.transition_to_state("ENDING")
                else:
                    self.speak_response("더 도와드릴 것이 있나요?")
                    self.transition_to_state("LISTENING_FOR_REQUEST")

            elif self.current_state == "GENERAL_CONVERSATION":
                self.publisher_.publish(String(data="State,GeneralConversation"))
                # 일반 대화 로직 처리
                self.publisher_.publish(String(data="Gesture,ProcessingText"))
                response = self.get_reply(self.request_text)
                self.speak_response(response)
                self.transition_to_state("LISTENING_FOR_REQUEST")

            elif self.current_state == "ENDING":
                goodbye_message = "감사합니다. 즐거운 쇼핑 되십시오."
                self.speak_response(goodbye_message)  # 종료 인사
                self.publisher_.publish(String(data="Navigation,Return"))  # 종료 및 복귀 제스처 발행
                # 필요한 종료 처리 수행
                sys.exit()

        except Exception as e:
            self.get_logger().error(f"An error occurred in state {self.current_state}: {str(e)}")


    def transition_to_state(self, new_state):
        self.get_logger().info(f'Transitioning from {self.current_state} to {new_state}')
        self.get_logger().info(f"현재 상태: {self.current_state}, 새 상태: {new_state}")
        state_transition_msg = f"[State][{self.current_state} to {new_state}]"
        self.publisher_.publish(String(data=state_transition_msg))
        self.current_state = new_state

    def greet_customer(self):
        self.get_logger().info(f'인사 메시지: "{self.intro_msg}"')
        self.publisher_.publish(String(data=self.intro_msg))
        self.transition_to_state("CONFIRMING_SERVICE_USE")

    def speak_response(self, response_text):
        try:
            synthesis_input = texttospeech.SynthesisInput(text=response_text)
            voice = texttospeech.VoiceSelectionParams(
                language_code="ko-KR", name='ko-KR-Standard-D', ssml_gender=texttospeech.SsmlVoiceGender.MALE
            )
            audio_config = texttospeech.AudioConfig(
                audio_encoding=texttospeech.AudioEncoding.MP3
            )

            response = self.text_to_speech_client.synthesize_speech(
                input=synthesis_input, voice=voice, audio_config=audio_config
            )

            audio_file = tempfile.mktemp(suffix='.mp3')
            with open(audio_file, 'wb') as out:
                out.write(response.audio_content)
            self.get_logger().info(f"오디오 파일 생성됨: {audio_file}")

            # 오디오 파일 재생
            pygame.mixer.init()
            pygame.mixer.music.load(audio_file)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy() == True:
                continue
            self.get_logger().info(f"오디오 파일 재생 완료: {audio_file}")

            return audio_file
        except Exception as e:
            self.get_logger().error(f"speak_response 메소드에서 오류 발생: {str(e)}")
            return None

    def record_audio(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
        print("녹음 시작")
        frames = []

        for _ in range(0, int(16000 / 1024 * 5)):
            try:
                data = stream.read(1024)
                frames.append(data)
            except Exception as e:
                print(f"An error occurred while reading the audio stream: {str(e)}")
                break
        print("녹음 종료")

        stream.stop_stream()
        stream.close()
        p.terminate()

        filepath = tempfile.mktemp(suffix=".wav")
        try:
            with wave.open(filepath, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
                wf.setframerate(16000)
                wf.writeframes(b''.join(frames))
        except Exception as e:
            print(f"An error occurred while writing the audio data to the file: {str(e)}")

        return filepath

    def transcribe_file_v2(self, audio_file_path):
        try:
            with io.open(audio_file_path, "rb") as audio_file:
                content = audio_file.read()

            audio = speech.RecognitionAudio(content=content)
            config = speech.RecognitionConfig(
                encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=16000,
                language_code="ko-KR"
            )

            response = self.client.recognize(config=config, audio=audio)

            for result in response.results:
                print("인식 결과: {}".format(result.alternatives[0].transcript))

        except Exception as e:
            print(f"오류 발생: {e}")
            # 또는 self.get_logger().error(f"오류 발생: {e}") 사용
            return None

        finally:
            # 오디오 파일 삭제
            os.remove(audio_file_path)

        return response.results[0].alternatives[0].transcript if response.results else ""


    def generate_response(self, request_text):
        try:
            response = openai.Completion.create(
                engine="text-davinci-003",
                request_text=request_text,
                max_tokens=50,
                temperature=0.7,
                n=1,
                stop=None
            )
            return response.choices[0].text.strip()
        except Exception as e:
            self.get_logger().error(f"OpenAI GPT-3 응답 생성 중 오류 발생: {str(e)}")
            return "죄송합니다, 응답을 생성하는 데 문제가 발생했습니다."
        
    def get_reply(self, prompt):
        model_name = "gpt-3.5-turbo"
        messages = [
            {"role": "system", "content": "너는 백화점 안내원 라이언이야."},
            {"role": "system", "content": "너는 백화점 손님을 안내할 것이고, 존칭을 사용해야해. 공손한 말투를 사용해줘."},
            {"role": "system", "content": "대화에서 화장실에 대해 물어보면 화장실은 문 밖에 바로 있다고 설명해."},
            {"role": "system", "content": "대화에서 신체 부위 중 하나인 발에 대해  신발이 있는 곳으로 안내할지 물어봐."},
            {"role": "system", "content": "남성 의류에 대해서 물어보면 의류가 있는 곳으로 안내해드릴지 물어봐."},
            {"role": "user", "content": prompt}
        ]

        response = openai.ChatCompletion.create(
          model=model_name,
          messages=messages
        )

        return response.choices[0].message["content"]

    def speak_long_response(self, filename):
        if os.path.exists(filename):
            with open(filename, 'r', encoding='utf-8') as file:
                long_response = file.read()
            self.speak_response(long_response)
        else:
            self.get_logger().error(f"파일 {filename}을 찾을 수 없습니다.")

    def get_keyword_explanation(self, keyword):
        base_path = "/home/geuny/feature_ws/src/hyrian_gpthri/text"
        filename = os.path.join(base_path, f"{keyword}_explain.txt")

        if os.path.exists(filename):
            with open(filename, 'r', encoding='utf-8') as file:
                return file.read()
        else:
            return "해당 키워드에 대한 설명이 없습니다."

        
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    
    def identify_keyword(self, text):
        # 예시: 특정 키워드 식별 로직
        keywords = ["옷", "신발", "전자기기", "책"]
        for keyword in keywords:
            if keyword in text:
                return keyword
        return None

def main(args=None):
    rclpy.init(args=args)
    print("Starting ROS node")
    state_machine = ConversationStateMachine()

    def run_state_machine():
        while rclpy.ok():
            state_machine.run_state_machine()

    state_machine_thread = threading.Thread(target=run_state_machine)
    state_machine_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(state_machine, timeout_sec=0.1)

    except Exception as e:
        print(f"An error occurred: {str(e)}")

    finally:
        state_machine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


