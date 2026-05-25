## Define build and run commands for SITL framework

build:
	cmake -S flight-computer -B flight-computer/build
	cmake --build flight-computer/build

run:
	./flight-computer/build/FlightComputer & \
	python3 environment/main.py

sitl:
	$(MAKE) build
	$(MAKE) run

clean:
	rm -rf flight-computer/build