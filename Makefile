
all: driver_make

driver_make:
	@cd driver;\
	make -s

install:
	@cd driver;\
	make install -s

clean:
	@cd driver;\
	make clean -s

remove:
	@cd driver;\
	make remove -s
