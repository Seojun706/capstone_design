#!/usr/bin/env python3

import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk
from PIL import Image, ImageTk
import os, pygame, threading, datetime, time, json, openai, subprocess
import speech_recognition as sr 
from gtts import gTTS

# openai는 0.28로 다운로드
# pip install openai==0.28

# speech_recognition 라이브러리 3.10.0 버전으로 다운그레이드
# pip install --upgrade SpeechRecognition == 3.10.0

# ----------------------------------------------------------

# 메인 프로그램
class HospitalApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("양념감자 캡스톤디자인")
        self.geometry("1600x2560")
        self.attributes("-fullscreen", True)
        self.resizable(False,False)

        self.nanum_font = tkfont.Font(family="NanumBarunGothic", size=25)
        self.password_verified = False # 비밀번호 인증 상태 변수 초기화

        self.frames = {}
        self.create_frames()
        self.show_frame("MainPage")

    def create_frames(self):

        login_page = LoginPage(self)
        self.frames["LoginPage"] = login_page

        register_page = RegisterPage(self, login_page)
        self.frames["RegisterPage"] = register_page

        for F in (InitialPage, MainPage, DoctorPage, TalkingPage, MapPage, FAQPage, DeliveryPage, ListPage, PasswordPage, WorkDeliveryPage, InitialMovingPage):
            frame = F(self)
            self.frames[F.__name__] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        # LoginPage와 RegisterPage를 마지막에 grid에 추가
        login_page.grid(row=0, column=0, sticky="nsew")
        register_page.grid(row=0, column=0, sticky="nsew")

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

    def show_frame(self, page_name):
        frame = self.frames[page_name]

        if page_name == "ListPage":
            frame.update_patient_buttons()

        if page_name == "InitialPage":
            frame.start_voice_interaction()
        frame.tkraise()

        if page_name == "TalkingPage":
            frame.start_voice_interaction()

        if page_name == "DoctorPage":
            frame.start_voice_interaction()

    def open_initial_page(self):
        self.show_frame("InitialPage")

    def open_doctor_page(self):
        self.show_frame("DoctorPage")

    def open_talking_page(self):
        self.show_frame("TalkingPage")

    def open_delivery_page(self):
        self.show_frame("DeliveryPage")

    def open_map_page(self):
        self.show_frame("MapPage")

    def open_faq_page(self):
        self.show_frame("FAQPage")

    def open_list_page(self):
        self.show_frame("ListPage")

    def open_password_page(self):
        self.show_frame("PasswordPage")

    def open_login_page(self):
        self.show_frame("LoginPage")

    def open_register_page(self):
        self.show_frame("RegisterPage")

# 바탕화면 설정
def set_background(root, path):
    img = Image.open(path)
    background_img = ImageTk.PhotoImage(img)
    background_label = tk.Label(root, image=background_img)
    background_label.place(x=0, y=0, relwidth=1, relheight=1)
    background_label.image = background_img

# ----------------------------------------------------------

# 초기 화면
class InitialPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self,master)
        self.master = master
        self.set_background_click("./image/bg_initial.png")
        self.voice_active = False
        self.voice_thread = None

    # 바탕화면 설정 (클릭이 되는)
    def set_background_click(self, path):
        img = Image.open(path)
        background_img = ImageTk.PhotoImage(img)
        background_label = tk.Label(self, image=background_img)
        background_label.place(x=0, y=0, relwidth=1, relheight=1)
        background_label.image = background_img
        background_label.bind("<Button-1>", self.on_click)

    def on_click(self, event):
        self.voice_active = False
        self.master.show_frame("MainPage")

    def start_voice_interaction(self):
        if not self.voice_active:
            self.voice_active = True
            if not self.voice_thread or not self.voice_thread.is_alive():
                self.voice_thread = threading.Thread(target=self.speak_voice, daemon=True)
                self.voice_thread.start()
                

    def stop_voice_interaction(self):
        self.voice_active = False

    def speak_voice(self):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            recognizer.adjust_for_ambient_noise(source, duration=1) # 배경 소음 조절
            while self.voice_active:
                try:
                    audio = recognizer.listen(source)
                    command = recognizer.recognize_google(audio, language='ko-KR').lower()
                    print("1")
                    if "감자" in command:
                        print("2")
                        self.speak_text("네, 무엇을 도와드릴까요?")
                        print("3")
                        while True:
                            try:
                                audio = recognizer.listen(source)
                                time.sleep(1)
                                print("4")
                                command = recognizer.recognize_google(audio, language='ko-KR').lower()
                                print("5")
                                break  # 인식에 성공하면 루프를 종료
                            except sr.UnknownValueError:
                                print("왜못들어")
                                continue 
                    
                        if "내과" in command:
                            self.speak_text("내꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("내과로 이동")
                            subprocess.run(["python", "./move/move_robot_naegwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "흉부외과" in command:
                            self.speak_text("흉부외꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("흉부외과로 이동")
                            subprocess.run(["python", "./move/move_robot_hyungbugwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "정형외과" in command:
                            self.speak_text("정형외꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("정형외과로 이동")
                            subprocess.run(["python", "./move/move_robot_jeonghyeonggwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "외과" in command:
                            self.speak_text("외꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("외과로 이동")
                            subprocess.run(["python", "./move/move_robot_waegwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "이비인후과" in command:
                            self.speak_text("이비인후꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("이비인후과로 이동")
                            subprocess.run(["python", "./move/move_robot_ibiinhugwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "소아과" in command:
                            self.speak_text("소아꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("소아과로 이동")
                            subprocess.run(["python", "./move/move_robot_soahgwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "응급" in command:
                            self.speak_text("응급의학꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("응급의학과로 이동")
                            subprocess.run(["python", "./move/move_robot_eunggeupgwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "정신" in command:
                            self.speak_text("정신의학꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("정신의학과로 이동")
                            subprocess.run(["python", "./move/move_robot_jeongshingwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "피부" in command:
                            self.speak_text("피부꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("피부과로 이동")
                            subprocess.run(["python", "./move/move_robot_pibugwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "신경" in command:
                            self.speak_text("신경꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("신경과로 이동")
                            subprocess.run(["python", "./move/move_robot_singyeonggwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "가정" in command:
                            self.speak_text("가정의학꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("가정의학과로 이동")
                            subprocess.run(["python", "./move/move_robot_gajeonggwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "접수" in command:
                            self.speak_text("접수처로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("접수처로 이동")
                            subprocess.run(["python", "./move/move_robot_geopsu.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "원무" in command:
                            self.speak_text("원무꽈로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("원무과로 이동")
                            subprocess.run(["python", "./move/move_robot_wonmugwa.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "엘리베이터" in command:
                            self.speak_text("엘리베이터 앞으로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("엘리베이터로 이동")
                            subprocess.run(["python", "./move/move_robot_elevator.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "남자" in command and "화장실" in command:
                            self.speak_text("남자화장실 앞으로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("화장실로 이동")
                            subprocess.run(["python", "./move/move_robot_toilet_male.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                        elif "여자" in command and "화장실" in command:
                            self.speak_text("여자화장실 앞으로 안내해드릴게요. 저를 따라오세요")
                            self.stop_voice_interaction()
                            print("화장실로 이동")
                            subprocess.run(["python", "./move/move_robot_toilet_female.py"])
                            self.master.show_frame("InitialMovingPage")
                            return
                except sr.UnknownValueError:
                    continue
                except sr.RequestError:
                    self.speak_text("음성 서비스에 문제가 발생했습니다")
                    continue

    # TTS 읽기
    def speak_text(self, text):
        tts = gTTS(text=text, lang='ko')
        filename = "response.mp3"
        tts.save(filename)

        pygame.mixer.init()
        pygame.mixer.music.load(filename)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            time.sleep(0.1)

        pygame.mixer.music.unload()
        pygame.mixer.quit()
        os.remove(filename)


class InitialMovingPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.master = master
        self.set_background_click("./image/bg_moving.png")
    
    def set_background_click(self, path):
        img = Image.open(path)
        background_img = ImageTk.PhotoImage(img)
        background_label = tk.Label(self, image=background_img)
        background_label.place(x=0, y=0, relwidth=1, relheight=1)
        background_label.image = background_img
        background_label.bind("<Button-1>", self.on_click)
        
    def on_click(self, event):
        self.master.show_frame("InitialPage")


# 메인 화면
class MainPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        set_background(self, "./image/bg_blur.png")
        
        self.nanum_font = tkfont.Font(family="NanumBarunGothic", size=15)
        self.create_widgets()

    def create_widgets(self):
        self.create_main_top_bar()

        images = {
            "doctor": "./image/doctor.png",
            "talking": "./image/talking.png",
            "map": "./image/map.png",
            "faq": "./image/faq.png",
            "delivery": "./image/delivery.png",
            "list": "./image/list.png",
        }

        resized_images = {}
        for key, path in images.items():
            img = Image.open(path)
            resized_img = img.resize((400, 400), Image.LANCZOS)
            resized_images[key] = ImageTk.PhotoImage(resized_img)

        buttons_info = [
            (resized_images["doctor"], "간이 진료보기", self.master.open_doctor_page),
            (resized_images["faq"], "자주하는 질문", self.master.open_faq_page),
            (resized_images["delivery"], "심부름하기", self.master.open_delivery_page),
            (resized_images["talking"], "대화하기", self.master.open_talking_page),
            (resized_images["map"], "병원 지도", self.master.open_map_page),
            (resized_images["list"], "환자 차트", self.master.open_list_page),
        ]

        self.buttons = []

        button_size = 450
        button_gap = (1600 - 2 * button_size) / 3
        text_height = 30
        spacing_below_text = 20

        vertical_offset = 150

        for i, (image, text, command) in enumerate(buttons_info):
            col, row = divmod(i, 3)
            x = button_gap + col * (button_size + button_gap)
            y = button_gap + row * (button_size + text_height + spacing_below_text + 150) + vertical_offset

            button = self.create_image_button_with_text(image, text, command, x, y, button_size, text_height)
            self.buttons.append(button)
        
        # 관리자 모드 버튼
        password_button = tk.Button(self, text="관리자모드", font=self.nanum_font, command=self.master.open_password_page)
        password_button.place(relx=0.99, rely=0.99, anchor="se")

        # 로그인 버튼
        login_img = Image.open("./image/login.png")
        login_photo = ImageTk.PhotoImage(login_img.resize((100, 100), Image.LANCZOS))
        login_button = tk.Button(self, image=login_photo, bg="#CDF0FF", borderwidth=0, highlightthickness=0, activebackground="#CDF0FF", command=self.master.open_login_page)
        login_button.image = login_photo
        login_button.place(x=20, y=80) # 왼쪽 상단

        # "로그인" 텍스트
        login_label = tk.Label(self, text="로그인", font=self.master.nanum_font, bg="#CDF0FF")
        login_label.place(x=150, y=110)

        self.update_buttons_state()

    def create_image_button_with_text(self, image, text, command, x, y, button_size, text_height):
        shadow_offset = 5

        shadow = tk.Label(self, bg="black")
        shadow.place(x = x + shadow_offset, y = y + shadow_offset, width = button_size, height = button_size)

        # 이미지 버튼 생성
        button = tk.Button(self, image=image, bg="white", borderwidth=2, relief="solid", command=command)
        button.place(x=x, y=y, width=button_size, height=button_size)
        button.image = image

        # 텍스트 레이블 생성
        label_y = y + button_size + 10 # 텍스트의 y좌표
        label = tk.Label(self, text=text, bg='white', font=self.master.nanum_font)
        label.place(x=x, y=label_y, width=button_size, height=text_height)

        return button
    
    def create_main_top_bar(self):
        top_bar = tk.Frame(self, bg="#F2F2F2", height=50)
        top_bar.pack(side = "top", fill="x")

        # 홈 버튼
        home_img = Image.open("./image/home_yellow.png")
        home_photo = ImageTk.PhotoImage(home_img.resize((40, 40), Image.LANCZOS))
        home_button = tk.Button(top_bar, image=home_photo, bg="#F2F2F2", command=self.master.open_initial_page)
        home_button.image = home_photo
        home_button.pack(side = "right", padx=(20,30), pady=10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side = "left", padx=20, pady=10)
        update_time(self.time_label)

        # 관리자 모드 텍스트
        self.admin_mode_label = tk.Label(top_bar, text="", bg="#F2F2F2", fg="black", font=self.master.nanum_font)
        self.admin_mode_label.pack(side="right", padx=(0, 10), pady=10)

    def update_buttons_state(self):
        if self.master.password_verified:
            self.buttons[2].config(state=tk.NORMAL)
            self.buttons[5].config(state=tk.NORMAL)
            self.admin_mode_label.config(text="[관리자 모드]")
        else:
            self.buttons[2].config(state=tk.DISABLED)
            self.buttons[5].config(state=tk.DISABLED)
            self.admin_mode_label.config(text="")

# ----------------------------------------------------------

# 로그인 페이지
class LoginPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        set_background(self, "./image/bg_blur.png")

        self.nanum_large_font = tkfont.Font(family="NanumBarunGothic", size=35)
        self.create_widgets()

        self.master.login_success = False # 로그인 성공 여부 변수
        self.master.logged_in_user = None # 로그인한 사용자 이름 저장 변수
        

    def create_widgets(self):
        self.create_top_bar()
        
        # 이름 선택 콤보박스
        name_label = tk.Label(self, text="이름", font=self.nanum_large_font, bg="#CDF0FF")
        name_label.place(relx=0.4, rely=0.2, anchor="e")
        self.name_combobox = ttk.Combobox(self, font=self.nanum_large_font)
        self.name_combobox.place(relx=0.5, rely=0.2, anchor="w", width=200)
        self.load_names()

        # 아이디
        id_label = tk.Label(self, text="아이디", font=self.nanum_large_font, bg="#CDF0FF")
        id_label.place(relx=0.4, rely=0.3, anchor="e")
        self.id_value = tk.Entry(self, text="", font=self.nanum_large_font, width=15)
        self.id_value.place(relx=0.5, rely=0.3, anchor="w", x=10)

        # 비밀번호
        password_label = tk.Label(self, text="비밀번호", font=self.nanum_large_font, bg="#CDF0FF")
        password_label.place(relx=0.4, rely=0.4, anchor="e")
        self.password_value = tk.Entry(self, text="", font=self.nanum_large_font, width=15, show="*")
        self.password_value.place(relx=0.5, rely=0.4, anchor="w", x=10)


        # 가입, 로그인, 로그아웃 버튼
        button_frame = tk.Frame(self, bg="#CDF0FF")
        button_frame.place(relx=0.5, rely=0.5, anchor="center")

        register_button = tk.Button(button_frame, text="가입", font=self.nanum_large_font, command=self.master.open_register_page)
        register_button.grid(row=0, column=0, padx=10)

        loginin_button = tk.Button(button_frame, text="로그인", font=self.nanum_large_font, command=self.login)
        loginin_button.grid(row=0, column=1, padx=10)

        logout_button = tk.Button(button_frame, text="로그아웃", font=self.nanum_large_font, command=self.logout)
        logout_button.grid(row=0, column=2, padx=10)

        self.virtual_keyboard()

    def load_names(self):
        files = os.listdir("./patient")
        names = [file.replace(".json", "") for file in files if file.endswith(".json")]
        self.name_combobox['values'] = names

    def virtual_keyboard(self):

        self.nanum_bigger_font = tkfont.Font(family="NanumBarunGothic", size=25)

        keys = [
            ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],
            ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P'],
            ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L'],
            ['Z', 'X', 'C', 'V', 'B', 'N', 'M','<-']
        ]

        keyboard_frame = tk.Frame(self, bg="#CDF0FF")
        keyboard_frame.place(relx=0.5, rely=0.7, anchor="n")

        for row_index, row_keys in enumerate(keys):
            for col_index, col_key in enumerate(row_keys):
                if col_key == '<-':
                    button = tk.Button(keyboard_frame, text=col_key, font=self.nanum_bigger_font, width=5, height=3, command=self.backspace)
                else:
                    button = tk.Button(keyboard_frame, text=col_key, font=self.nanum_bigger_font, width=5, height=3, command=lambda k=col_key: self.insert_text(k.lower()))
                button.grid(row=row_index, column=col_index, padx=4, pady=4)
    
        for col_index in range(len(keys[0])):
            keyboard_frame.grid_columnconfigure(col_index, weight=1)

    def insert_text(self, char):
        if self.id_value.focus_get() == self.id_value:
            self.id_value.insert(tk.END, char)
        elif self.password_value.focus_get() == self.password_value:
            self.password_value.insert(tk.END, char)
        else:
            return

    def backspace(self):
        if self.id_value.focus_get() == self.id_value:
            current_text = self.id_value.get()
            self.id_value.delete(0, tk.END)
            self.id_value.insert(0, current_text[:-1])
        elif self.password_value.focus_get() == self.password_value:
            current_text = self.password_value.get()
            self.password_value.delete(0, tk.END)
            self.password_value.insert(0, current_text[:-1])
        else:
            return

    def login(self):
        selected_name = self.name_combobox.get()
        entered_id = self.id_value.get()
        entered_password = self.password_value.get()

        if not selected_name or not entered_id or not entered_password:
            messagebox.showwarning("경고", "입력정보란을 확인해주세요")
            return

        try:
            with open(f"./patient/{selected_name}.json", "r", encoding="utf-8") as file:
                data = json.load(file)
        except FileNotFoundError:
            messagebox.showerror("에러", "사용자 파일을 찾을 수 없습니다.")
            return

        if entered_id == data["id"] and entered_password == data["password"]:
            self.master.frames["MainPage"].admin_mode_label.config(text=f"{data['FamilyName']}{data['GivenName']}님, 환영합니다!")
            self.master.login_success = True
            self.master.logged_in_user = f"{data['FamilyName']}{data['GivenName']}"
            self.clear_entries()
            self.master.show_frame("MainPage")
        else:
            messagebox.showerror("에러", "아이디 혹은 비밀번호가 일치하지 않습니다.")

    def logout(self):
        if self.master.login_success:
            self.master.login_success = False
            self.master.logged_in_user = None
            self.clear_entries()
            self.master.frames["MainPage"].admin_mode_label.config(text="")
            self.master.show_frame("MainPage")

    def clear_entries(self):
        self.name_combobox.set("")
        self.id_value.delete(0, tk.END)
        self.password_value.delete(0, tk.END)

    def create_top_bar(self):
        # 상단바 생성
        top_bar = tk.Frame(self, bg="#F2F2F2", height = 50)
        top_bar.pack(side = "top", fill="x")

        # 돌아가기 버튼
        back_img = Image.open("./image/left_arrow.png")
        back_photo = ImageTk.PhotoImage(back_img.resize((40,40), Image.LANCZOS))
        back_button = tk.Button(top_bar, image=back_photo, bg="#F2F2F2", command = lambda: self.master.show_frame("MainPage"))
        back_button.image = back_photo
        back_button.pack(side = "left", padx=(20,10), pady=10)

        # 돌아가기 텍스트
        back_label = tk.Label(top_bar, text="돌아가기", bg="#F2F2F2", font=self.master.nanum_font)
        back_label.pack(side="left", pady= 10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side = "right", padx= 20, pady = 10)
        update_time(self.time_label)

# 가입 페이지
class RegisterPage(tk.Frame):
    def __init__(self, master, login_page):
        tk.Frame.__init__(self, master)
        self.login_page = login_page
        set_background(self, "./image/bg_blur.png")

        self.nanum_large_font = tkfont.Font(family="NanumBarunGothic", size=30)
        self.create_widgets()

    def create_widgets(self):
        self.create_top_bar()

        # 이름
        name_label = tk.Label(self, text="이름", font=self.nanum_large_font, bg="#CDF0FF")
        name_label.place(relx=0.4, rely=0.1, anchor="e")
        self.name_value = tk.Entry(self, text="", font=self.nanum_large_font, width=15)
        self.name_value.place(relx=0.5, rely=0.1, anchor="w", x=10)

        # 아이디
        id_label = tk.Label(self, text="아이디", font=self.nanum_large_font, bg="#CDF0FF")
        id_label.place(relx=0.4, rely=0.15, anchor="e")
        self.id_value = tk.Entry(self, text="", font=self.nanum_large_font, width=15)
        self.id_value.place(relx=0.5, rely=0.15, anchor="w", x=10)

        # 비밀번호
        password_label = tk.Label(self, text="비밀번호", font=self.nanum_large_font, bg="#CDF0FF")
        password_label.place(relx=0.4, rely=0.2, anchor="e")
        self.password_value = tk.Entry(self, text="", font=self.nanum_large_font, width=15)
        self.password_value.place(relx=0.5, rely=0.2, anchor="w", x=10)

        # 생년월일
        birth_label = tk.Label(self, text="생년월일", font=self.nanum_large_font, bg="#CDF0FF")
        birth_label.place(relx=0.4, rely=0.25, anchor="e")
        self.birth_value = tk.Entry(self, text="", font=self.nanum_large_font, width=15)
        self.birth_value.place(relx=0.5, rely=0.25, anchor="w", x=10)

        # 성별
        gender_label = tk.Label(self, text="전화번호", font=self.nanum_large_font, bg="#CDF0FF")
        gender_label.place(relx=0.4, rely=0.3, anchor="e")

        self.gender_var = tk.StringVar(value=None)
        male_radiobutton = tk.Radiobutton(self, text="남자", variable=self.gender_var, value="남자", font=self.nanum_large_font, bg="#CDF0FF")
        male_radiobutton.place(relx=0.5, rely=0.3, anchor="w")

        female_radiobutton = tk.Radiobutton(self, text="여자", variable=self.gender_var, value="여자", font=self.nanum_large_font, bg="#CDF0FF")
        female_radiobutton.place(relx=0.6, rely=0.3, anchor="w")

        # 전화번호
        phone_label = tk.Label(self, text="전화번호", font=self.nanum_large_font, bg="#CDF0FF")
        phone_label.place(relx=0.4, rely=0.35, anchor="e")
        self.phone_value = tk.Entry(self, text="", font=self.nanum_large_font, width=15)
        self.phone_value.place(relx=0.5, rely=0.35, anchor="w", x=10)

        # 신장
        height_label = tk.Label(self, text="신장 [cm]", font=self.nanum_large_font, bg="#CDF0FF")
        height_label.place(relx=0.4, rely=0.4, anchor="e")
        self.height_value = tk.Entry(self, text="", font=self.nanum_large_font, width=15)
        self.height_value.place(relx=0.5, rely=0.4, anchor="w", x=10)

        # 체중
        weight_label = tk.Label(self, text="체중 [kg]", font=self.nanum_large_font, bg="#CDF0FF")
        weight_label.place(relx=0.4, rely=0.45, anchor="e")
        self.weight_value = tk.Entry(self, text="", font=self.nanum_large_font, width=15)
        self.weight_value.place(relx=0.5, rely=0.45, anchor="w", x=10)

        # 혈액형
        blood_label = tk.Label(self, text="혈액형", font=self.nanum_large_font, bg="#CDF0FF")
        blood_label.place(relx=0.4, rely=0.5, anchor="e")

        blood_types = ["A", "B", "AB", "O", "A (Rh-)", "B (Rh-)", "AB (Rh-)", "O (Rh-)"]
        self.blood_combobox = ttk.Combobox(self, values=blood_types, font=self.nanum_large_font)
        self.blood_combobox.place(relx=0.5, rely=0.5, anchor="w", width=200)


        register_button = tk.Button(self, text="가입", font=self.nanum_large_font, command=self.register)
        register_button.place(relx=0.45, rely=0.6, anchor="w", width=200)

    
    def register(self):
        name = self.name_value.get()
        id_value = self.id_value.get()
        password = self.password_value.get()
        birth = self.birth_value.get()
        gender = self.gender_var.get()
        phone = self.phone_value.get()
        height = self.height_value.get()
        weight = self.weight_value.get()
        blood = self.blood_combobox.get()

        # 이름에서 성과 이름 분리
        if len(name) == 2:
            family_name = name[0]
            given_name = name[1:]
        elif len(name) == 3:
            family_name = name[0]
            given_name = name[1:]
        elif len(name) == 4:
            family_name = name[:2]
            given_name = name[2:]

        # JSON 데이터 생성
        data = {
            "FamilyName": family_name,
            "GivenName": given_name,
            "id": id_value,
            "password": password,
            "Birth": birth,
            "Gender": gender,
            "PhoneNumber": phone,
            "Height": height,
            "Weight": weight,
            "BloodType": blood,
            "Symptoms": [],
            "Diagnosis": [],
            "Treatment": [],
            "Additional_Question": []
        }

        # JSON 파일 저장
        file_path = f"./patient/{name}.json"
        with open(file_path, 'w', encoding='utf-8') as json_file:
            json.dump(data, json_file, ensure_ascii=False, indent=4)

        self.master.show_frame("LoginPage")
        self.login_page.load_names()
        self.reset()

    def reset(self):
        self.name_value.delete(0, tk.END)
        self.id_value.delete(0, tk.END)
        self.password_value.delete(0, tk.END)
        self.birth_value.delete(0, tk.END)
        self.gender_var.set(None)
        self.phone_value.delete(0, tk.END)
        self.height_value.delete(0, tk.END)
        self.weight_value.delete(0, tk.END)
        self.blood_combobox.set('')

    def create_top_bar(self):
        # 상단바 생성
        top_bar = tk.Frame(self, bg="#F2F2F2", height = 50)
        top_bar.pack(side = "top", fill="x")

        # 돌아가기 버튼
        back_img = Image.open("./image/left_arrow.png")
        back_photo = ImageTk.PhotoImage(back_img.resize((40,40), Image.LANCZOS))
        back_button = tk.Button(top_bar, image=back_photo, bg="#F2F2F2", command = lambda: self.master.show_frame("LoginPage"))
        back_button.image = back_photo
        back_button.pack(side = "left", padx=(20,10), pady=10)

        # 돌아가기 텍스트
        back_label = tk.Label(top_bar, text="돌아가기", bg="#F2F2F2", font=self.master.nanum_font)
        back_label.pack(side="left", pady= 10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side = "right", padx= 20, pady = 10)
        update_time(self.time_label)

# ----------------------------------------------------------

# 간이 진료 페이지
class DoctorPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        set_background(self, "./image/bg_blur.png")
        self.doctor_assistant = DoctorAssistant(self)
        self.voice_thread = None
        self.voice_active = False
        self.create_widgets()
        self.current_y = 0

        self.nanum_smaller_font = tkfont.Font(family="NanumBarunGothic", size=15)


    def create_widgets(self):
        self.create_top_bar()

        self.chat_frame = tk.Frame(self, bg="#ADD8E6")
        self.chat_frame.place(relx=0.5, rely=0.55, anchor='center', relwidth=0.65, relheight=0.75)
        self.chat_canvas = tk.Canvas(self.chat_frame, bg="#ADD8E6")
        self.chat_scrollbar = tk.Scrollbar(self.chat_frame, orient="vertical", command=self.chat_canvas.yview)
        self.chat_canvas.configure(yscrollcommand=self.chat_scrollbar.set)
        self.chat_scrollbar.pack(side="right", fill="y")
        self.chat_canvas.pack(side="left", fill="both", expand=True)

        self.chat_content_frame = tk.Frame(self.chat_canvas, bg="#ADD8E6")
        self.chat_canvas.create_window((0,0), window=self.chat_content_frame, anchor="nw")

        custom_font = tkfont.Font(family="NanumBarunGothic", size=20)

        self.start_chat_label = tk.Label(self, text="진료를 시작하려면 '안녕하세요'라고 인사해보세요.", font=custom_font, bg="#CDF0FF")
        self.start_chat_label.pack(pady=50)

        self.guide_chat_label = tk.Label(self, text="", font=custom_font, bg="#CDF0FF")
        self.guide_chat_label.pack(pady=20)

    def update_scroll_region(self):
        self.chat_canvas.configure(scrollregion=self.chat_canvas.bbox("all"))

    def start_voice_interaction(self):
        if not self.voice_thread or not self.voice_thread.is_alive():
            self.voice_active = True
            self.voice_thread = threading.Thread(target=self.speak_voice, daemon=True)
            self.voice_thread.start()

    def speak_voice(self):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            recognizer.adjust_for_ambient_noise(source, duration=1)
            while self.voice_active:
                try:
                    audio = recognizer.listen(source)
                    self.update_guide_chat_label(True)
                    command = recognizer.recognize_google(audio, language='ko-KR').lower()
                    self.update_guide_chat_label(False)
                    if  command == "안녕하세요":
                        print("진료시작")
                        self.display_user_message(command)
                        if self.master.login_success:
                            user_name = self.master.logged_in_user
                            response = f"안녕하세요 {user_name}님, 어떻게 도와드릴까요?"
                        else:
                            response = "안녕하세요, 어떻게 도와드릴까요?"
                        self.display_bot_message(response)
                        print(f"[AIDoctor] : {response}")
                        self.speak_text(response)
                        self.start_chat_label.pack_forget()
                        self.doctor_assistant.interact_with_user()
                        break
                except sr.UnknownValueError:
                    continue
                except sr.RequestError:
                    continue

    def update_guide_chat_label(self, listening):
        if listening:
            self.guide_chat_label.config(text="듣고있어요...")
        else:
            self.guide_chat_label.config(text="")

    def speak_text(self, text):
        tts = gTTS(text=text, lang='ko')
        filename = "output.mp3"
        tts.save(filename)

        pygame.mixer.init()
        pygame.mixer.music.load(filename)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            continue

        pygame.mixer.music.unload()
        pygame.mixer.quit()

        os.remove(filename)

    # 대화 초기화
    def reset_interaction(self):
        self.voice_thread = None
        self.start_chat_label.pack()

    def stop_voice_interaction(self):
        self.voice_active = False

    # 유저 메시지 출력
    def display_user_message(self, text):
        user_frame = tk.Frame(self.chat_canvas, bg="#FFFF00")
        user_label = tk.Label(user_frame, text=text, bg="#FFFF00", font=self.nanum_smaller_font, wraplength=250)
        user_label.pack(side="right", fill="both", expand=True, padx=(0, 20))

        # 프레임의 크기를 업데이트하고 위치 계산
        user_frame.update_idletasks()
        frame_height = user_frame.winfo_reqheight()
        canvas_width = self.chat_canvas.winfo_width()
        frame_width = user_frame.winfo_reqheight()

        self.chat_canvas.create_window((canvas_width - frame_width - 10, self.current_y), window=user_frame, anchor="ne")

        # 다음 위젯의 y 좌표를 업데이트
        self.current_y += frame_height

        self.update_scroll_region()
        self.chat_canvas.yview_moveto(1.0)

    def display_bot_message(self, text):
        bot_frame = tk.Frame(self.chat_canvas, bg="#FFFFFF")
        bot_label = tk.Label(bot_frame, text=text, bg="#FFFFFF", font=self.nanum_smaller_font, wraplength=250)
        bot_label.pack(side="left", fill="both", expand=True, padx=(20, 0))

        # 프레임 크기 업데이트 위치 계산
        bot_frame.update_idletasks()
        frame_height = bot_frame.winfo_reqheight()

        self.chat_canvas.create_window((10, self.current_y), window=bot_frame, anchor="nw")

        self.current_y += frame_height

        self.update_scroll_region()
        self.chat_canvas.yview_moveto(1.0)

    def reset_chat(self):
        for widget in self.chat_canvas.winfo_children():
            widget.destroy()
        self.current_y = 0

    def create_top_bar(self):
        # 상단바 생성
        top_bar = tk.Frame(self, bg="#F2F2F2", height = 50)
        top_bar.pack(side = "top", fill="x")

        # 돌아가기 버튼
        back_img = Image.open("./image/left_arrow.png")
        back_photo = ImageTk.PhotoImage(back_img.resize((40,40), Image.LANCZOS))
        back_button = tk.Button(top_bar, image=back_photo, bg="#F2F2F2", command = lambda: [self.stop_voice_interaction(), self.reset_interaction_and_return()])
        back_button.image = back_photo
        back_button.pack(side = "left", padx=(20,10), pady=10)

        # 돌아가기 텍스트
        back_label = tk.Label(top_bar, text="돌아가기", bg="#F2F2F2", font=self.master.nanum_font)
        back_label.pack(side="left", pady= 10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side = "right", padx= 20, pady = 10)
        update_time(self.time_label)

    # 돌아가기 버튼을 누르면 MainPage로 돌아가고 대화내역 초기화
    def reset_interaction_and_return(self):
        self.reset_chat()
        self.reset_interaction()
        self.master.show_frame("MainPage")

class DoctorAssistant:
    def __init__(self, doctor_page):
        self.doctor_page = doctor_page
        openai.api_key = "api_key"
        self.conversations = []

    def interact_with_user(self):
        self.conversations.append("[Patient] 안녕하세요")
        
        if self.doctor_page.master.login_success:
            user_name = self.doctor_page.master.logged_in_user
            self.conversations.append(f"[AIDoctor] 안녕하세요 {user_name}님, 어떻게 도와드릴까요?")
        else:
            self.conversations.append("[AIDoctor] 안녕하세요, 어떻게 도와드릴까요?")

        messages = [
            {"role": "system", "content": "You work as a chatbot doctor at a general hospital, treating patients. Based on the patient's symptoms, provide clear and positive responses, and respond concisely to ensure the conversation is easy to understand. When asking the patient questions, ask only one question at a time. Do not inform the patient of any suspected diseases or diagnoses from the first question; instead, provide this information after obtaining sufficient answers about the patient's symptoms during the consultation. Help decide and implement the best treatment methods based on the diagnosis results. Respond with one sentence whenever possible. If the patient asks about a specific medication, provide information about that medication. Since you already work at the hospital, guide the patient on which department to visit for a consultation rather than recommending a hospital visit."}
        ]

        while True:
            with sr.Microphone() as source:
                recognizer = sr.Recognizer()
                recognizer.adjust_for_ambient_noise(source, duration=1)
                self.doctor_page.update_guide_chat_label(True)
                audio = recognizer.listen(source)
                text = self.transcribe_audio_to_text(audio, recognizer)
                self.doctor_page.update_guide_chat_label(False)
                
                if text:
                    self.conversations.append(f"[Patient] {text}")
                    self.doctor_page.display_user_message(text)
                    print(f"[Patient] : {text}")

                    if "종료" in text:
                        if self.doctor_page.master.login_success:
                            save_question = "지금까지의 진료 내용을 저장할까요?" # json에 저장한다는뜻
                            self.doctor_page.display_bot_message(save_question)
                            self.speak_text(save_question)

                            self.doctor_page.update_guide_chat_label(True)
                            audio = recognizer.listen(source)
                            answer = self.transcribe_audio_to_text(audio, recognizer)
                            self.doctor_page.update_guide_chat_label(False)
                            print(f"[Patient] : {answer}")
                            self.doctor_page.display_user_message(answer)

                            negative_response = ['괜찮아', '하지마']
                            # negative_response = ['아니', '괜찮아', '하지마', '됐어', '안 해도 돼', '싫어', '아니오', '아니요', '괜찮아요', '하지마세요' , '괜찮습니다', '싫어요', '싫습니다', '안 해도 돼요', '됐어요']

                            if answer in negative_response:
                                farewell_message = "진료내용을 저장하지 않고 종료할게요. 좋은 하루 보내세요!"
                            else:
                                self.save_conversations()
                                self.analysis_diagnosis()
                                farewell_message = "진료내용을 저장했어요. 좋은 하루 보내세요!"
                                self.send_email()

                        else:
                            self.save_conversations()
                            farewell_message = "진료를 종료합니다. 좋은 하루 보내세요!"

                        self.doctor_page.display_bot_message(farewell_message)
                        print(f"[AIDoctor] : {farewell_message}")
                        self.speak_text(farewell_message)
                        self.doctor_page.voice_active = False
                        break

                    response = self.generate_response(text, messages)
                    self.conversations.append(f"[AIDoctor] {response}")
                    self.doctor_page.display_bot_message(response)
                    print(f"[AIDoctor] : {response}")
                    self.speak_text(response)
                    self.doctor_page.update_guide_chat_label(True)

    def send_email(self):
        if self.doctor_page.master.login_success:
            user_name = self.doctor_page.master.logged_in_user
            user_json_path = f"./patient/{user_name}.json"
            os.system(f"python send_email.py {user_json_path}")

    def save_conversations(self):
        directory = "./diagnosis/"

        if not os.path.exists(directory):
            os.makedirs(directory)

        if self.doctor_page.master.login_success:
            user_name = self.doctor_page.master.logged_in_user
            filename = f"{user_name}_Diag"
        else:
            filename = "Unknown_Diag"

        file_extension = ".txt"
        file_count = 1

        while os.path.exists(f"{directory}{filename}{file_count}{file_extension}"):
            file_count += 1

        self.diagnosis_file = f"{directory}{filename}{file_count}{file_extension}"
        with open(self.diagnosis_file, "w", encoding="utf-8") as file:
            for line in self.conversations:
                if "종료" not in line:
                    file.write(line + "\n")

        self.conversations.clear()

    def analysis_diagnosis(self):
        with open(self.diagnosis_file, 'r', encoding='utf-8') as file:
            diagnosis_text = file.read()

        analysis = Analysis(diagnosis_text)
        analysis.analyze_diagnosis()

        if self.doctor_page.master.login_success:
            user_name = self.doctor_page.master.logged_in_user
            user_json_path = f"./patient/{user_name}.json"
            analysis.save_summary_to_json(user_json_path)

    def transcribe_audio_to_text(self, audio_data, recognizer):
        try:
            return recognizer.recognize_google(audio_data, language='ko-KR')
        except Exception:
            pass

    def generate_response(self, text, messages):
        messages.append({"role": "user", "content": text})

        response = openai.ChatCompletion.create(
            model = "gpt-4-turbo",
            messages = messages,
            max_tokens = 200,
            temperature = 0.4
        )
        response_text = response["choices"][0]["message"]["content"]
        messages.append({"role": "assistant", "content": response_text})

        return response_text
    
    def speak_text(self, text):
        tts = gTTS(text=text, lang='ko')
        filename = "output.mp3"
        tts.save(filename)

        pygame.mixer.init()
        pygame.mixer.music.load(filename)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            continue

        pygame.mixer.music.unload()
        pygame.mixer.quit()

        os.remove(filename)

class Analysis:
    def __init__(self, diagnosis_text):
        self.diagnosis_text = diagnosis_text
        self.summary = None

    def analyze_diagnosis(self):

        messages = [
            {"role": "system", "content": "You are an AI that reviews the consultation details between a patient and an AI doctor and provides a summary. The patient's questions and answers are prefixed with '[Patient]', and the AI doctor's questions and answers are prefixed with '[AIDoctor]'. Extract key points from the conversation under the headings 'patient_symptoms', 'possible_diseases', 'AI_doctor_prescriptions', and 'additional_questions'. Do not change the names of the headings. Write the summary using simple keywords or terms within double quotes. Do not add any subheadings under 'patient_symptoms', 'possible_diseases', 'AI_doctor_prescriptions', 'additional_questions', only add the summarized text. The body of the text file is in Korean, so you need to recognize it in Korean."},
            {"role": "user", "content": self.diagnosis_text}
        ]

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages = messages,
            max_tokens = 500,
            temperature = 0.2
        )

        self.summary = response.choices[0].message['content']
        print("Analysis Summary:\n", self.summary)

    def save_summary_to_json(self, user_json_path):
        if self.summary is None:
            raise ValueError("Summary has not been generated yet. Call analyze_diagnosis first.")
        
        summary_dict = self.parse_text_summary(self.summary)

        with open(user_json_path, 'r+', encoding='utf-8') as file:
            user_data = json.load(file)

            if summary_dict.get('patient_symptoms'):
                user_data['Symptoms'] = summary_dict.get('patient_symptoms', [])
            if summary_dict.get('possible_diseases'):
                user_data['Diagnosis'] = summary_dict.get('possible_diseases', [])
            if summary_dict.get('AI_doctor_prescriptions'):
                user_data['Treatment'] = summary_dict.get('AI_doctor_prescriptions', [])
            if summary_dict.get('additional_questions'):
                user_data['Additional_Question'] = summary_dict.get('additional_questions', [])

            file.seek(0)
            json.dump(user_data, file, ensure_ascii=False, indent=4)
            file.truncate()
        print("Updated User Data:", user_data)
    
    def parse_text_summary(self, text_summary):
        summary_dict = {}
        lines = text_summary.split('\n')
        for line in lines:
            if line.startswith("**patient_symptoms**:"):
                summary_dict['patient_symptoms'] = line.replace("**patient_symptoms**:", "").strip()
            elif line.startswith("**possible_diseases**:"):
                summary_dict['possible_diseases'] = line.replace("**possible_diseases**:", "").strip()
            elif line.startswith("**AI_doctor_prescriptions**:"):
                summary_dict['AI_doctor_prescriptions'] = line.replace("**AI_doctor_prescriptions**:", "").strip()
            elif line.startswith("**additional_questions**:"):
                summary_dict['additional_questions'] = line.replace("**additional_questions**:", "").strip()
        return summary_dict

# ----------------------------------------------------------

# 자유 대화 페이지
class TalkingPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        set_background(self, "./image/bg_blur.png")
        self.voice_assistant = VoiceAssistant(self)
        self.voice_thread = None
        self.voice_active = False
        self.create_widgets()
        self.current_y = 0


    def create_widgets(self):
        self.create_top_bar()

        self.chat_frame = tk.Frame(self, bg="#ADD8E6")
        self.chat_frame.place(relx=0.5, rely=0.55, anchor='center', relwidth=0.65, relheight=0.75)
        self.chat_canvas = tk.Canvas(self.chat_frame, bg="#ADD8E6")
        self.chat_scrollbar = tk.Scrollbar(self.chat_frame, orient="vertical", command=self.chat_canvas.yview)
        self.chat_canvas.configure(yscrollcommand=self.chat_scrollbar.set)
        self.chat_scrollbar.pack(side="right", fill="y")
        self.chat_canvas.pack(side="left", fill="both", expand=True)

        self.chat_content_frame = tk.Frame(self.chat_canvas, bg="#ADD8E6")
        self.chat_canvas.create_window((0,0), window=self.chat_content_frame, anchor="nw")

        custom_font = tkfont.Font(family="NanumBarunGothic", size=20)

        self.start_chat_label = tk.Label(self, text="대화를 시작하려면 '안녕' 이라고 인사해보세요.\n대화를 종료하려면 '종료'라고 말하세요.", font=custom_font, bg="#ADD8E6")
        self.start_chat_label.pack(pady=50)

    def update_scroll_region(self):
        self.chat_canvas.configure(scrollregion=self.chat_canvas.bbox("all"))

    def start_voice_interaction(self):
        if not self.voice_thread or not self.voice_thread.is_alive():
            self.voice_active = True
            self.voice_thread = threading.Thread(target=self.speak_voice, daemon=True)
            self.voice_thread.start()

    def speak_voice(self):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            recognizer.adjust_for_ambient_noise(source, duration=1)
            while self.voice_active:
                try:
                    audio = recognizer.listen(source)
                    command = recognizer.recognize_google(audio, language='ko-KR').lower()
                    if "안녕" in command:
                        print("인삿말 인식")
                        self.display_user_message(command)

                        if self.master.login_success:
                            user_name = self.master.logged_in_user
                            response = f"안녕하세요 {user_name}님, 오늘은 어떤 대화를 해볼까요?"
                        else:
                            response = "안녕하세요, 오늘은 어떤 대화를 해볼까요?"
                        self.display_bot_message(response)
                        print(f"[감자] : {response}")
                        self.speak_text(response)
                        self.start_chat_label.pack_forget()
                        self.voice_assistant.interact_with_user()
                        break
                except sr.UnknownValueError:
                    continue
                except sr.RequestError:
                    continue

    def speak_text(self, text):
        tts = gTTS(text=text, lang='ko')
        filename = "output.mp3"
        tts.save(filename)

        pygame.mixer.init()
        pygame.mixer.music.load(filename)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            continue

        pygame.mixer.music.unload()
        pygame.mixer.quit()

        os.remove(filename)


    # 대화 초기화
    def reset_interaction(self):
        self.voice_thread = None
        self.start_chat_label.pack()

    def stop_voice_interaction(self):
        self.voice_active = False

    # 유저 메시지 출력
    def display_user_message(self, text):
        user_frame = tk.Frame(self.chat_canvas, bg="#FFFF00")
        user_label = tk.Label(user_frame, text=text, bg="#FFFF00", font=self.master.nanum_font, wraplength=250)
        user_label.pack(side="right", fill="both", expand=True, padx=(0, 20))

        # 프레임의 크기를 업데이트하고 위치 계산
        user_frame.update_idletasks()
        frame_height = user_frame.winfo_reqheight()
        canvas_width = self.chat_canvas.winfo_width()
        frame_width = user_frame.winfo_reqheight()

        self.chat_canvas.create_window((canvas_width - frame_width - 10, self.current_y), window=user_frame, anchor="ne")

        # 다음 위젯의 y 좌표를 업데이트
        self.current_y += frame_height

        self.update_scroll_region()
        self.chat_canvas.yview_moveto(1.0)

    def display_bot_message(self, text):
        bot_frame = tk.Frame(self.chat_canvas, bg="#FFFFFF")
        bot_label = tk.Label(bot_frame, text=text, bg="#FFFFFF", font=self.master.nanum_font, wraplength=250)
        bot_label.pack(side="left", fill="both", expand=True, padx=(20, 0))

        # 프레임 크기 업데이트 위치 계산
        bot_frame.update_idletasks()
        frame_height = bot_frame.winfo_reqheight()

        self.chat_canvas.create_window((10, self.current_y), window=bot_frame, anchor="nw")

        self.current_y += frame_height

        self.update_scroll_region()
        self.chat_canvas.yview_moveto(1.0)

    def reset_chat(self):
        for widget in self.chat_canvas.winfo_children():
            widget.destroy()
        self.current_y= 0


    def create_top_bar(self):
        # 상단바 생성
        top_bar = tk.Frame(self, bg="#F2F2F2", height = 50)
        top_bar.pack(side = "top", fill="x")

        # 돌아가기 버튼
        back_img = Image.open("./image/left_arrow.png")
        back_photo = ImageTk.PhotoImage(back_img.resize((40,40), Image.LANCZOS))
        back_button = tk.Button(top_bar, image=back_photo, bg="#F2F2F2", command = lambda: [self.stop_voice_interaction(), self.reset_interaction_and_return()])
        back_button.image = back_photo
        back_button.pack(side = "left", padx=(20,10), pady=10)

        # 돌아가기 텍스트
        back_label = tk.Label(top_bar, text="돌아가기", bg="#F2F2F2", font=self.master.nanum_font)
        back_label.pack(side="left", pady= 10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side = "right", padx= 20, pady = 10)
        update_time(self.time_label)

    # 돌아가기 버튼을 누르면 MainPage로 돌아가고 대화내역 초기화
    def reset_interaction_and_return(self):
        self.reset_chat()
        self.reset_interaction()
        self.master.show_frame("MainPage")

class VoiceAssistant:
    def __init__(self, talking_page):
        self.talking_page = talking_page
        openai.api_key = "api_key"

    def interact_with_user(self):
        messages = [
            {"role": "system", "content": "You are an AI chatbot designed to provide assistance and emotional stability to patients and the elderly visiting the hospital. Whenever patients within the hospital start a conversation, this AI chatbot must respond sensitively to their emotional and social needs, offering kind, empathetic, and encouraging dialogue. It serves as a friend they can talk to when feeling lonely. Avoid complex explanations and provide clear, concise, and positive guidance. To keep the conversation easy to understand, responses should ideally be limited to one or two sentences."}
        ]

        while True:
            with sr.Microphone() as source:
                recognizer = sr.Recognizer()
                recognizer.adjust_for_ambient_noise(source, duration=1)
                audio = recognizer.listen(source)
                text = self.transcribe_audio_to_text(audio, recognizer)
                
                if text:
                    self.talking_page.display_user_message(text)
                    print(f"[사용자] : {text}")

                    if "종료" in text:
                        farewell_message = "대화를 종료합니다. 좋은 하루 보내세요!"
                        self.talking_page.display_bot_message(farewell_message)
                        print(f"[감자] : {farewell_message}")
                        self.speak_text(farewell_message)
                        self.talking_page.voice_active = False
                        break

                    response = self.generate_response(text, messages)
                    self.talking_page.display_bot_message(response)
                    print(f"[감자] : {response}")
                    self.speak_text(response)

    def transcribe_audio_to_text(self, audio_data, recognizer):
        try:
            return recognizer.recognize_google(audio_data, language='ko-KR')
        except Exception:
            pass

    def generate_response(self, text, messages):
        messages.append({"role": "user", "content": text})

        response = openai.ChatCompletion.create(
            model = "gpt-4-turbo",
            messages = messages,
            max_tokens = 200,
            temperature = 0.3
        )
        response_text = response["choices"][0]["message"]["content"]
        messages.append({"role": "assistant", "content": response_text})

        return response_text
    
    def speak_text(self, text):
        tts = gTTS(text=text, lang='ko')
        filename = "output.mp3"
        tts.save(filename)

        pygame.mixer.init()
        pygame.mixer.music.load(filename)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            continue

        pygame.mixer.music.unload()
        pygame.mixer.quit()

        os.remove(filename)

# ----------------------------------------------------------

# 관리자모드 비밀번호 페이지
class PasswordPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.create_widgets()
        self.password = ""

    def create_widgets(self):
        set_background(self, "./image/bg_blur.png")
        self.create_top_bar()

        button_width = 100
        button_height = 100

        # 비밀번호가 입력되는 Entry
        self.password_entry = tk.Entry(self, font=('Arial', 24), show='*', width=4, justify='center')
        self.password_entry.place(relx=0.5, rely=0.3, anchor='center', relwidth=0.6)
        
        # 키패드
        keypad = [
            ('7', 0.4, 0.4), ('8', 0.5, 0.4), ('9', 0.6, 0.4),
            ('4', 0.4, 0.5), ('8', 0.5, 0.5), ('6', 0.6, 0.5),
            ('1', 0.4, 0.6), ('2', 0.5, 0.6), ('3', 0.6, 0.6),
            ('0', 0.4, 0.7), ('Enter', 0.6, 0.7)
        ]
        for key, x, y in keypad:
            button = tk.Button(self, text=key, font=('Arial', 24), command=lambda k=key: self.press_key(k))
            button.place(width=button_width, height=button_height, relx=x, rely=y, anchor='center')
    
    def create_top_bar(self):
        # 상단바 생성
        top_bar = tk.Frame(self, bg="#F2F2F2", height = 50)
        top_bar.pack(side = "top", fill="x")

        # 돌아가기 버튼
        back_img = Image.open("./image/left_arrow.png")
        back_photo = ImageTk.PhotoImage(back_img.resize((40,40), Image.LANCZOS))
        back_button = tk.Button(top_bar, image=back_photo, bg="#F2F2F2", command = lambda: (self.reset_password_entry(), self.master.show_frame("MainPage")))
        back_button.image = back_photo
        back_button.pack(side = "left", padx=(20,10), pady=10)

        # 돌아가기 텍스트
        back_label = tk.Label(top_bar, text="돌아가기", bg="#F2F2F2", font=self.master.nanum_font)
        back_label.pack(side="left", pady= 10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side = "right", padx= 20, pady = 10)
        update_time(self.time_label)

    def press_key(self, key):
        if self.password_entry.get() == "비밀번호가 틀렸습니다!":
            self.reset_password_entry()

        if key == 'Enter':
            self.check_password()
        else:
            self.password += key
            self.password_entry.insert('end', key)
    
    def check_password(self):
        if self.password == '1234':
            if self.master.password_verified:
                self.master.password_verified = False
            else:
                self.master.password_verified = True
                self.master.login_success = False
                self.master.logged_in_user = None
                self.master.frames["MainPage"].admin_mode_label.config(text="")
            self.master.frames["MainPage"].update_buttons_state()
            self.master.show_frame("MainPage")
            self.reset_password_entry()
        else:
            self.password_entry.delete(0, 'end')
            self.password_entry.config(show='')
            self.password_entry.insert('end', "비밀번호가 틀렸습니다!")

    def reset_password_entry(self):
        self.password = ""
        self.password_entry.delete(0, 'end')
        self.password_entry.config(show='*')

# ----------------------------------------------------------

# 심부름 페이지 (관리자모드 on)
class DeliveryPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        set_background(self, "./image/bg_blur.png")
        
        self.create_widgets()
        self.awaiting_confirmation = False
        self.room_number = ""

    def create_widgets(self):
        self.create_top_bar()

        button_width = 100
        button_height = 100

        self.current_entry = tk.Label(self, font=('Arial', 24), bg='white')
        self.current_entry.place(relx=0.5, rely=0.2, anchor='center', relwidth=0.6)

        # 비밀번호가 입력되는 Entry
        self.room_entry = tk.Entry(self, font=('Arial', 24), width=4, justify='center')
        self.room_entry.place(relx=0.5, rely=0.3, anchor='center', relwidth=0.6)
        
        # 키패드
        keypad = [
            ('7', 0.4, 0.4), ('8', 0.5, 0.4), ('9', 0.6, 0.4),
            ('4', 0.4, 0.5), ('5', 0.5, 0.5), ('6', 0.6, 0.5),
            ('1', 0.4, 0.6), ('2', 0.5, 0.6), ('3', 0.6, 0.6),
            ('0', 0.4, 0.7), ('-', 0.5, 0.7), ('Enter', 0.6, 0.7)
        ]
        for key, x, y in keypad:
            button = tk.Button(self, text=key, font=('Arial', 24), command=lambda k=key: self.press_key(k))
            button.place(width=button_width, height=button_height, relx=x, rely=y, anchor='center')

        # 이동 버튼
        move_button = tk.Button(self, text="이동", font=('Arial', 24), command=self.move_to_room)
        move_button.place(width=button_width, height=button_height, relx=0.5, rely=0.8, anchor='center')

    def create_top_bar(self):
        # 상단바 생성
        top_bar = tk.Frame(self, bg="#F2F2F2", height=50)
        top_bar.pack(side="top", fill="x")

        # 돌아가기 버튼
        back_img = Image.open("./image/left_arrow.png")
        back_photo = ImageTk.PhotoImage(back_img.resize((40, 40), Image.LANCZOS))
        back_button = tk.Button(top_bar, image=back_photo, bg="#F2F2F2", command=lambda: self.master.show_frame("MainPage"))
        back_button.image = back_photo
        back_button.pack(side="left", padx=(20, 10), pady=10)

        # 돌아가기 텍스트
        back_label = tk.Label(top_bar, text="돌아가기", bg="#F2F2F2", font=self.master.nanum_font)
        back_label.pack(side="left", pady=10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side="right", padx=20, pady=10)
        update_time(self.time_label)

    def press_key(self, key):
        if self.awaiting_confirmation and key != 'Enter':
            self.current_entry.config(text="")
            self.awaiting_confirmation = False
        
        if key == 'Enter':
            self.check_room()
        else:
            current_text = self.room_entry.get()
            self.room_entry.delete(0, tk.END)
            self.room_entry.insert(0, current_text + key)

    def check_room(self):
        valid_rooms = [
            '201', '202-1', '202-2', '202-3', '202-4', '203', '204', '208',
            '209', '210', '211', '212', '213', '214'
        ]
        room_number = self.room_entry.get()

        if room_number in valid_rooms:
            self.room_number = room_number
            self.current_entry.config(text=f"{room_number}호로 이동할까요?")
            self.awaiting_confirmation = True
        else:
            self.current_entry.config(text="존재하지 않는 호실입니다!")
            self.awaiting_confirmation = False
        self.room_entry.delete(0, tk.END)

    def move_to_room(self):
        if self.awaiting_confirmation and self.room_number:
            self.create_work_delivery_page()
            self.awaiting_confirmation = False
            self.current_entry.config(text="")

    def create_work_delivery_page(self):
        if self.room_number == '201':
            subprocess.run(["python", "move_robot_soahgwa.py"])
        elif self.room_number == '202-1':
            subprocess.run(["python", "move_robot_naegwa.py"])
        elif self.room_number == '202-2':
            subprocess.run(["python", "move_robot_waegwa.py"])
        elif self.room_number == '202-3':
            subprocess.run(["python", "move_robot_ibiinhugwa.py"])
        elif self.room_number == '202-4':
            subprocess.run(["python", "move_robot_jeonghyeonggwa.py"])
        elif self.room_number == '208':
            subprocess.run(["python", "move_robot_wonmugwa.py"])
        elif self.room_number == '209':
            subprocess.run(["python", "move_robot_gajeonggwa.py"])
        elif self.room_number == '210':
            subprocess.run(["python", "move_robot_hyungbugwa.py"])
        elif self.room_number == '211':
            subprocess.run(["python", "move_robot_singyeonggwa.py"])
        elif self.room_number == '212':
            subprocess.run(["python", "move_robot_pibugwa.py"])
        elif self.room_number == '213':
            subprocess.run(["python", "move_robot_jeongshingwa.py"])
        elif self.room_number == '214':
            subprocess.run(["python", "move_robot_eunggeupgwa.py"])
        else:
            pass

        self.master.frames["WorkDeliveryPage"].start_delivery()
        self.master.show_frame("WorkDeliveryPage")


class WorkDeliveryPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.master = master
        set_background(self, "./image/bg_delivery.png")

        stop_button = tk.Button(self, text="중지", font=self.master.nanum_font, command=self.stop_delivery)
        stop_button.pack(side="bottom", pady=80)
        self.delivery_thread = None
        self.stop_event = threading.Event()

    def start_delivery(self):
        self.stop_event.clear()
        self.delivery_thread = threading.Thread(target=self.play_audio, daemon=True)
        self.delivery_thread.start()

    def play_audio(self):
        pygame.mixer.init()
        while not self.stop_event.is_set():
            tts = gTTS(text="심부름 중이에요", lang='ko')
            filename = "delivery.mp3"
            tts.save(filename)
            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                if self.stop_event.is_set():
                    pygame.mixer.music.stop()
                    break
                time.sleep(1)
            pygame.mixer.music.unload()
            os.remove(filename)
            if self.stop_event.is_set():
                break
            time.sleep(10)
        pygame.mixer.quit()

    def stop_delivery(self):
        self.stop_event.set()
        pygame.mixer.music.stop()  # 즉시 오디오 재생 중지
        self.master.show_frame("DeliveryPage")
        if self.delivery_thread is not None:
            self.delivery_thread.join()


# 환자 차트 페이지 (관리자모드 on)
class ListPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        set_background(self, "./image/bg_blur.png")

        self.nanum_diagnosis_font = tkfont.Font(family="NanumBarunGothic", size=10)
        
        self.patient_buttons = []
        self.create_widgets()
        self.current_patient = None

    def create_widgets(self):
        self.create_top_bar()

        self.top_frame = tk.Frame(self, bg="#ADD8E6")
        self.top_frame.pack(side="top", fill="x")

        self.canvas = tk.Canvas(self.top_frame, bg="#ADD8E6")
        self.canvas.pack(side="left", fill="both", expand=True)
        
        self.button_frame = tk.Frame(self.canvas, bg="#ADD8E6")
        self.canvas.create_window((0, 0), window=self.button_frame, anchor="nw")

        self.bottom_frame = tk.Frame(self, bg="#ADD8E6")
        self.bottom_frame.pack(side="top", fill="both", expand=True)

        self.left_frame = tk.Frame(self.bottom_frame, bg="#F0F0F0")
        self.left_frame.pack(side="left", fill="y", padx=10, pady=10)

        self.right_frame = tk.Frame(self.bottom_frame, bg="#F0F0F0")
        self.right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        self.create_patient_details()

        self.top_scrollbar = ttk.Scrollbar(self.top_frame, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.top_scrollbar.set)
        self.top_scrollbar.pack(side="right", fill="y")

        self.update_patient_buttons()

    def update_button_frame(self):
        self.button_frame.update_idletasks()
        self.canvas.config(scrollregion=self.canvas.bbox("all"))

    def update_patient_buttons(self):
        for button in self.patient_buttons:
            button.destroy()
        self.patient_buttons.clear()

        self.create_patient_buttons()
        self.update_button_frame()

    def create_patient_buttons(self):
        files = os.listdir("./patient")
        row = 0
        col = 0
        for file in files:
            if file.endswith(".json"):
                patient_name = file.replace(".json", "")
                button = tk.Button(self.button_frame, text=patient_name, font=self.master.nanum_font, command=lambda name=patient_name: self.load_patient_details(name))
                button.grid(row=row, column=col, padx=5, pady=5)
                self.patient_buttons.append(button)
                col += 1
                if col == 5:
                    col = 0
                    row += 1
        
    def create_patient_details(self):
        self.labels = {}

        # Left 프레임 디테일
        labels_texts = ["이름", "생년월일", "성별", "전화", "신장", "체중", "혈액형"]
        for i, text in enumerate(labels_texts):
            label = tk.Label(self.left_frame, text=text, font=self.master.nanum_font, bg="#F0F0F0")
            label.grid(row=i, column=0, sticky="e", padx=5, pady=40)
            value_label = tk.Label(self.left_frame, text="", font=self.master.nanum_font, bg="#F0F0F0", width=15)
            value_label.grid(row=i, column=1, sticky="w", padx=5, pady=40)
            self.labels[text] = value_label
        
        # Right 프레임 디테일
        labels_texts = ["환자증상", "진단내용", "치료방법", "추가질문"]
        for i, text in enumerate(labels_texts):
            label = tk.Label(self.right_frame, text=text, font=self.master.nanum_font, bg="#F0F0F0")
            label.grid(row=i*2, column=0, sticky="w", padx=5, pady=20) 
            value_label = tk.Label(self.right_frame, text="", font=self.master.nanum_font, bg="#DDE3E9", wraplength=500, width=30, height=6)
            value_label.grid(row=i*2+1, column=0, sticky="w", padx=5, pady=20) 
            self.labels[text] = value_label

        # 초기화 버튼
        reset_button = tk.Button(self.right_frame, text="초기화", font=self.master.nanum_font, command=self.reset_patient_details)
        reset_button.grid(row=8, column=0, padx=5, pady=20, columnspan=2)

    def load_patient_details(self, name):
        self.current_patient = name
        file_path = f"./patient/{name}.json"
        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)

        self.labels["이름"].config(text=f"{data['FamilyName']}{data['GivenName']}")
        self.labels["생년월일"].config(text=data["Birth"])
        self.labels["성별"].config(text=data["Gender"])
        self.labels["전화"].config(text=data["PhoneNumber"])
        self.labels["신장"].config(text=f"{data['Height']} cm")
        self.labels["체중"].config(text=f"{data['Weight']} kg")
        self.labels["혈액형"].config(text=f"{data['BloodType']} 형")
        self.labels["환자증상"].config(text="".join(data["Symptoms"]) if data["Symptoms"] else "")
        self.labels["진단내용"].config(text="".join(data["Diagnosis"]) if data["Diagnosis"] else "")
        self.labels["치료방법"].config(text="".join(data["Treatment"]) if data["Treatment"] else "")
        self.labels["추가질문"].config(text="".join(data["Additional_Question"]) if data["Additional_Question"] else "")

    def create_top_bar(self):
        # 상단바 생성
        top_bar = tk.Frame(self, bg="#F2F2F2", height = 50)
        top_bar.pack(side = "top", fill="x")

        # 돌아가기 버튼
        back_img = Image.open("./image/left_arrow.png")
        back_photo = ImageTk.PhotoImage(back_img.resize((40,40), Image.LANCZOS))
        back_button = tk.Button(top_bar, image=back_photo, bg="#F2F2F2", command = lambda: [self.clear_patient_details(), self.master.show_frame("MainPage"), self.update_patient_buttons()])
        back_button.image = back_photo
        back_button.pack(side = "left", padx=(20,10), pady=10)

        # 돌아가기 텍스트
        back_label = tk.Label(top_bar, text="돌아가기", bg="#F2F2F2", font=self.master.nanum_font)
        back_label.pack(side="left", pady= 10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side = "right", padx= 20, pady = 10)
        update_time(self.time_label)

    def clear_patient_details(self):
        for label in self.labels.values():
            label.config(text="")

    def reset_patient_details(self):
        if self.current_patient is None:
            return
        
        if all(label.cget("text") == "" for label in self.labels.values()):
            return
        
        response = messagebox.askokcancel("진료 내역 초기화", "진료 내역을 초기화하시겠습니까?")
        if response:
            file_path = f"./patient/{self.current_patient}.json"
            with open(file_path, 'r+', encoding='utf-8') as file:
                data = json.load(file)
                data["Symptoms"] = []
                data["Diagnosis"] = []
                data["Treatment"] = []
                data["Additional_Question"] = []
                file.seek(0)
                json.dump(data, file, ensure_ascii=False, indent=4)
                file.truncate()

            for text in ["환자증상", "진단내용", "치료방법", "추가질문"]:
                self.labels[text].config(text="")
            self.current_patient = None

# ----------------------------------------------------------

# 지도 페이지
class MapPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        set_background(self, "./image/bg_blur.png")
        
        self.create_widgets()

    def create_widgets(self):
        self.create_top_bar()

        map_img = Image.open("./image/floor2.png")
        map_photo = ImageTk.PhotoImage(map_img.resize((1600, 1178), Image.LANCZOS))

        self.map_label = tk.Label(self, image=map_photo)
        self.map_label.image = map_photo
        self.map_label.place(relx=0.5, rely=0.5, anchor="center")
        

    def create_top_bar(self):
        # 상단바 생성
        top_bar = tk.Frame(self, bg="#F2F2F2", height = 50)
        top_bar.pack(side = "top", fill="x")

        # 돌아가기 버튼
        back_img = Image.open("./image/left_arrow.png")
        back_photo = ImageTk.PhotoImage(back_img.resize((40,40), Image.LANCZOS))
        back_button = tk.Button(top_bar, image=back_photo, bg="#F2F2F2", command = lambda: self.master.show_frame("MainPage"))
        back_button.image = back_photo
        back_button.pack(side = "left", padx=(20,10), pady=10)

        # 돌아가기 텍스트
        back_label = tk.Label(top_bar, text="돌아가기", bg="#F2F2F2", font=self.master.nanum_font)
        back_label.pack(side="left", pady= 10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side = "right", padx= 20, pady = 10)
        update_time(self.time_label)

# 자주하는 질문 페이지
class FAQPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        set_background(self, "./image/bg_blur.png")
        
        self.create_widgets()

    def create_widgets(self):
        self.create_top_bar()

        button_font2 = tkfont.Font(family="NanumBarunGothic", size=30)

        self.faq_frame = tk.Frame(self, bg="#F2F2F2")
        self.faq_frame.place(relx=0.5, rely=0.4, anchor="center", relwidth=0.8, relheight=0.5)

        self.answer_label = tk.Label(self.faq_frame, text="", font=button_font2, bg="white", justify="left", wraplength=1200)
        self.answer_label.pack(side="bottom", fill="both", expand=True, pady=10)

        button1 = tk.Button(self.faq_frame, text="어떻게 회원가입/로그인을 하나요?", font=button_font2, command=self.show_answer1)
        button1.pack(side="top", fill="x", pady=10)

        button2 = tk.Button(self.faq_frame, text="어떻게 간이 진료를 볼 수 있나요?", font=button_font2, command=self.show_answer2)
        button2.pack(side="top", fill="x", pady=10)
        
        button3 = tk.Button(self.faq_frame, text="어떻게 길 안내를 받을 수 있나요?", font=button_font2, command=self.show_answer3)
        button3.pack(side="top", fill="x", pady=10)

    def create_top_bar(self):
        # 상단바 생성
        top_bar = tk.Frame(self, bg="#F2F2F2", height = 50)
        top_bar.pack(side = "top", fill="x")

        # 돌아가기 버튼
        back_img = Image.open("./image/left_arrow.png")
        back_photo = ImageTk.PhotoImage(back_img.resize((40,40), Image.LANCZOS))
        back_button = tk.Button(top_bar, image=back_photo, bg="#F2F2F2", command = lambda: self.master.show_frame("MainPage"))
        back_button.image = back_photo
        back_button.pack(side = "left", padx=(20,10), pady=10)

        # 돌아가기 텍스트
        back_label = tk.Label(top_bar, text="돌아가기", bg="#F2F2F2", font=self.master.nanum_font)
        back_label.pack(side="left", pady= 10)

        # 시간 표시 레이블
        self.time_label = tk.Label(top_bar, bg="#F2F2F2", font=self.master.nanum_font)
        self.time_label.pack(side = "right", padx= 20, pady = 10)
        update_time(self.time_label)

    def show_answer1(self):
        answer = ("왼쪽 위 로그인 버튼을 누르고,\n"
                "'가입' 버튼을 눌러 인적 사항을 적으시고,\n"
                "바로 이어지는 창에서 로그인을 하시면 됩니다.")
        self.answer_label.config(text=answer)

    def show_answer2(self):
        answer = ("로그인을 한 후 '간이진료보기'를 누르고,\n"
                "'안녕하세요'라고 인사하면 인공지능 의사와의 진료가 시작됩니다.\n\n"
                "진료 중에는 의심되는 증상을 직접 물어보시면\n의심 증상을 알려줍니다.\n\n"
                "진료를 끝내고 싶을 때는 '진료를 종료해주세요'와 같이\n"
                "'종료'라는 단어가 포함되도록 말해주세요.\n\n"
                "진료가 끝나고 '진료 내용을 저장할까요?' 라는 질문에\n"
                "'저장해주세요'와 같이 길게 대답해주세요.\n"
                "그러면 담당 의사에게 진료 내역이 전달되고,\n"
                "로봇 데이터베이스에 인공지능 의사와의 대화 내용이\n"
                "자동으로 요약되어 정리됩니다.")
        self.answer_label.config(text=answer)

    def show_answer3(self):
        answer = ("오른쪽 위 '홈' 버튼을 누르시고 '감자야' 라고 감자를 불러보세요.\n"
                "감자가 어디로 데려다드릴지 물어보게 될것입니다.\n"
                "내과, 이비인후과, 흉부외과, 엘리베이터,\n접수처 등 안내받고 싶은 곳으로\n"
                "안내해달라고 말하면, 그곳까지 감자가 안내해드릴거예요.")
        self.answer_label.config(text=answer)
# ----------------------------------------------------------


# 시간 표시
def update_time(label):
    now = datetime.datetime.now()
    date_format = "%Y년 %m월 %d일"
    hour_format = "%I"  # 12시간 형식
    am_pm = now.strftime("%p").lower()  # 오전/오후 정보
    am_pm_kr = "오전" if am_pm == "am" else "오후"

    formatted_time = now.strftime(f"{date_format}   |   {am_pm_kr} {hour_format}시 %M분 %S초").replace(" 0", " ")
    label.config(text = formatted_time)
    label.after(1000, update_time, label)

# 앱 실행
if __name__ == "__main__":
    app = HospitalApp()
    app.mainloop()