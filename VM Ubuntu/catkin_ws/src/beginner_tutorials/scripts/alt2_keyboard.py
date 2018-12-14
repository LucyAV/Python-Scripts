#!/usr/bin/env python
import keyboard

is_forward = False

def goforward():
	global is_forward
	is_forward = True
	print(is_forward)

def stopforward():
	global is_forward
	if not is_forward:
		return
	is_forward = False
	print(is_forward)

if __name__ == "__main__":
	is_forward = False
	stopforward()

	keyboard.add_hotkey('w', stopforward, trigger_on_release=True)
	keyboard.add_hotkey('w', goforward)
	
	keyboard.wait()