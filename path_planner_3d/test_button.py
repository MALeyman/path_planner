import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
import logging
logging.basicConfig(level=logging.INFO)

class TestWindow(QMainWindow):
	def __init__(self):
		super().__init__()
		central = QWidget()
		layout = QVBoxLayout(central)
		
		self.start_btn = QPushButton("Старт")
		self.start_btn.clicked.connect(self.start_test)
		layout.addWidget(self.start_btn)
		self.setCentralWidget(central)
	
	def start_test(self):
		logging.info("✅ КНОПКА РАБОТАЕТ!")
		print("ТЕСТ: БЕЗ ^C!")

app = QApplication(sys.argv)
w = TestWindow()
w.show()
sys.exit(app.exec_())
