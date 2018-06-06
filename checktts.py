try:
	import pyttsx
	print("pyttsx is installed.")
except ImportError, e:
	print("pyttsx is NOT installed. To install: sudo pip install pyttsx")