#!/usr/bin/env python
import keyboard
import time

forward = False

def key_press(key):
	if key.name is 'w':
		forward = True
		print("forward: ", forward)
	print("Pressed: ", key.name)

def key_release(key):
	if key.name is 'w':
		forward = False
		print("forward: ", forward)
	print("Released: ", key.name)

if __name__ == "__main__":
	keyboard.on_release(key_release)
	print("release")
	keyboard.on_press(key_press)
	print("press")

	while True:
		time.sleep(1)