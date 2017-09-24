# Richie Jarvis - http://nebul.ae
# 2017-09-24 - GPL licenced 

# novatherover-arduino-control
Arduino Control code for Nova the Rover

Nova the Rover (https://www.facebook.com/novatherover) is an Arduino controlled project I started a couple of years ago with a simple 24v Mobility Scooter brought off Ebay.  

I started off by rebuilding the chassis.  In hindsight I made more work for myself, but did learn to weld of-sorts.  In the first year of operation the contraption was a powered pull-cart/human carrying workhorse.  The steering was backwards at her first public outing, which made control interesting.  She was present at Lewes Bonfire Celebrations in 2016 lurking with the Bonfire Boyes, and got covered in red paper from firecrackers.

It was always my hope to convert the chassis into a remotely steerable unit, so I purchased a Lynxmotion PS2 controller (http://www.robotshop.com/uk/lynxmotion-ps2-controller-v4.html) and a cheap 12v 30mm/s Linear Actuator from Ebay (http://www.ebay.co.uk/itm/222620724976).


Originally I was hoping to use the original computer/controller unit that came with the Mobility scooter.  Unfortunately, that was not as easy as I first hoped, so I ditched that idea and purchased a 30A Cytron DC Motor Driver (http://www.robotshop.com/en/cytron-30a-5-30v-single-brushed-dc-motor-driver.html) to control the motors.

I then added an Arduino Uno (http://www.robotshop.com/uk/arduino-uno-r3-usb-microcontroller.html) and a 4 relay board to handle the switching of polarity, turning the magnetic brake on/off, etc to finish things off.

This project is the current set of control code as it develops over time.

Plans so far:
1. Replace on/off relay control on Linear Actuator with a proper ESC to give more granular steering.
2. Add weaponry :)
3. Add navigation lights, (loud) horn and control indicators
4. Add Distance sensor(s)
5. Add magnetic detector
6. Add camera
7. Add Raspberry Pi + intelligence(!?)
