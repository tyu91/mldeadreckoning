# alias for just everything
local_objs = gps.o TinyGPS++.o PCA9685.o icm20948.o MadgwickAHRS.o
linked_libs = -lwiringPi -lstdc++ -lpthread -lrt

imu_gps_log: imu_gps_log.o icm20948.o MadgwickAHRS.o gps.o TinyGPS++.o
	g++ imu_gps_log.o icm20948.o MadgwickAHRS.o gps.o TinyGPS++.o -o imu_gps_log $(linked_libs)

$(local_objs): %.o: %.cpp
	$(CC) -c $< -o $@

clean:
	rm *.o
	rm imu_gps_log