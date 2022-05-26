# Programa base para utilizar interfaz gráfica diseñada en QtDesigner

import sys
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication

import serial

class App(QMainWindow):
	def __init__(self):
		super().__init__()
		uic.loadUi("interfaz.ui", self)			# Ingresar nombre de su archivo .ui
		self.ser = serial.Serial(port="COM3", baudrate=9600, timeout=1.0)
		self.ser.close()
		self.pot1.valueChanged.connect(self.get_value_servo1)
		self.pot2.valueChanged.connect(self.get_value_servo2)
		self.pot3.valueChanged.connect(self.get_value_servo3)
		self.pot4.valueChanged.connect(self.get_value_servo4)

	def get_value_servo1(self):
		cont = 0
		valor = self.pot1.value()
		val = 1
		self.ser.open()									# Abrimos puerto serial
		self.label1.setText(str(valor))
		while cont < 2:
			if cont == 0:
				self.ser.write(chr(val).encode())			# Enviamos valor de la perilla como caracter
				y = val
			if cont == 1:
				self.ser.write(chr(valor).encode())			# Enviamos valor de la perilla como caracter
				y = valor
			cont = cont + 1
			a = self.ser.readline().decode()				# Leemos respuesta del PIC
			print(chr(y).encode())
		self.ser.close()								# Cerramos puerto serial

	def get_value_servo2(self):
		cont2 = 0
		valor2 = self.pot2.value()
		val2 = 2
		self.ser.open()
		self.label2.setText(str(self.pot2.value()))
		while cont2 < 2:
			if cont2 == 0:
				self.ser.write(chr(val2).encode())			# Enviamos valor de la perilla como caracter
				y = val2
			if cont2 == 1:
				self.ser.write(chr(valor2).encode())			# Enviamos valor de la perilla como caracter
				y = valor2
			cont2 = cont2 + 1
			a = self.ser.readline().decode()				# Leemos respuesta del PIC
			print(y)
		self.ser.close()

	def get_value_servo3(self):
		cont3 = 0
		valor3 = self.pot3.value()
		val3 = 3
		self.ser.open()
		self.label3.setText(str(self.pot3.value()))
		while cont3 < 2:
			if cont3 == 0:
				self.ser.write(chr(val3).encode())			# Enviamos valor de la perilla como caracter
				y = val3
			if cont3 == 1:
				self.ser.write(chr(valor3).encode())			# Enviamos valor de la perilla como caracter
				y = valor3
			cont3 = cont3 + 1
			a = self.ser.readline().decode()				# Leemos respuesta del PIC
			print(y)
		self.ser.close()

	def get_value_servo4(self):
		cont4 = 0
		valor4 = self.pot4.value()
		val4 = 4
		self.ser.open()
		self.label4.setText(str(self.pot4.value()))
		while cont4 < 2:
			if cont4 == 0:
				self.ser.write(chr(val4).encode())			# Enviamos valor de la perilla como caracter
				y = val4
			if cont4 == 1:
				self.ser.write(chr(valor4).encode())			# Enviamos valor de la perilla como caracter
				y = valor4
			cont4 = cont4 + 1
			a = self.ser.readline().decode()				# Leemos respuesta del PIC
			print(y)
		self.ser.close()


if __name__ == '__main__':
	app = QApplication(sys.argv)
	GUI = App()
	GUI.show()
	sys.exit(app.exec_())
