
# Heatlamp and avian detterance system "Thesis part 1"
# 1-Load-cell-sensor
This part is done using STM32 Microcontrollers with multiple actuators(heatlamp and a buzzer) and sensors like weight sensor (loadcell).the main object of this project
is to melt down the accumulated snow on the glass of the camera box.the process is initiated through a tempreture and humidity sensor that sense the present of the rain
above the glass of the camera box,once the sensor sense the present of the rain,the stm32 neucleo microcontroller initiate an electrical pulse that operate a certain  pin which is connected to an electronic relay that operate as an electronic switch for the operation of the heatlamp in accordings with the present of the rain.
# 2-Bird detterance system:
the bird detterance part is intigrated into the part of heatlamp,the purpose of this segment is to scare the birds away from standing on the glass.the process is done as the following, the ultrasonic sensor is positioned in the corner of the box which  is a distance measurment device.it measures the distance of the objects standing infront of it.when a bird or avians creatures stands on the box the ultrasonıc sensor will send an ultrasonic wave that will hit the object and return back to the sensor.by taking the lenght of the signal going forward and back we can cualculate the distance of the object standing on the glass of the camera box.
