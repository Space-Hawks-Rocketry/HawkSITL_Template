## Define build and run commands for SITL framework

build:
	cmake -S flight-computer -B flight-computer/build
	cmake --build flight-computer/build

run:
	-pkill -f FlightComputer || true
	-pkill -f "python3 environment/main.py" || true

	./flight-computer/build/FlightComputer & \
	python3 environment/main.py

sitl:
	$(MAKE) build
	$(MAKE) run

clean:
	rm -rf flight-computer/build