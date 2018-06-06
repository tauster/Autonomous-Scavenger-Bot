import numpy as np
import time

data_log = np.genfromtxt('qr_log.csv', delimiter=", ", dtype = "|S")

word_log = data_log[2:, 1]

word_log_cond = str(word_log[0])

for i in range(1, len(word_log)):
	if word_log[i - 1] != word_log[i]:
		word_log_cond = word_log_cond + " " + str(word_log[i])

while True:
	print word_log_cond
	time.sleep(0.5)