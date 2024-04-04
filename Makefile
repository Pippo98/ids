MAKEFLAGS += --no-print-directory

.PHONY: release debug format clean

debug:
	@if [ -d bin ]; then \
		echo "removing binaries"; \
		rm -r bin; \
	fi;
	mkdir -p debug; \
	cd debug; \
 	cmake .. -DCMAKE_BUILD_TYPE=Debug; \
	make -j${nproc};

release:
	@if [ -d bin ]; then \
		echo "removing binaries"; \
		rm -r bin; \
	fi;
	mkdir -p release; \
	cd release; \
	cmake .. -DCMAKE_BUILD_TYPE=Release; \
	make -j${nproc};

clean:
	@if [ -d .generated ]; then \
		echo "removing .generated folder"; \
		rm -r .generated;\
	fi;
	@if [ -d debug ]; then \
		echo "removing debug folder"; \
		rm -r debug;\
	fi;
	@if [ -d release ]; then \
		echo "removing release folder"; \
		rm -r release;\
	fi 