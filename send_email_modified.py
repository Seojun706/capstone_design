import smtplib, json, os, sys
from datetime import datetime
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

class SendEmail:
    def __init__(self, user_json_path):
        self.user_json_path = user_json_path
        self.from_address = ""    # 발신자 이메일
        self.password = ""   # 발신자 앱 비밀번호
        self.to_address = ['']  # 수신자 이메일


    def send(self):
        try:
            with open(self.user_json_path, 'r', encoding='utf-8') as file:
                user_data = json.load(file)

            user_name = f"{user_data['FamilyName']}{user_data['GivenName']}"
            user_gender = user_data['Gender']
            user_birth = user_data['Birth']
            user_phone = user_data['PhoneNumber']
            symptoms = user_data['Symptoms']
            diagnosis = user_data['Diagnosis']
            treatment = user_data['Treatment']
            additional_question = user_data['Additional_Question']

            now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            subject = f"[{now}] {user_name} 환자 AI 진단내역"
            body = f"환자이름: {user_name}\n환자성별: {user_gender}\n환자생년월일: {user_birth}\n전화번호: {user_phone}\n\n증상: {symptoms}\n진단: {diagnosis}\n치료: {treatment}\n추가질문: {additional_question}"

            msg = MIMEMultipart()
            msg['From'] = self.from_address
            msg['Subject'] = subject
            msg.attach(MIMEText(body, 'plain'))

            server = smtplib.SMTP('smtp.gmail.com', 587)
            server.starttls()
            server.login(self.from_address, self.password)
            text = msg.as_string()
            for to_address in self.to_address:
                msg['To'] = to_address
                server.sendmail(self.from_address, to_address, text)
            server.quit()
            print("Email sent successfully")
        except Exception as e:
            print(f"Failed to send email : {e}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        user_json_path = sys.argv[1]
        email_sender = SendEmail(user_json_path)
        email_sender.send()
    else:
        print("No user JSON path provided!")