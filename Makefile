PWD			:= $(shell pwd)
WORK_DIR	:= /var/tmp/beat

CMake = mkdir -p $(1); cd $(1); cmake $(PWD) -G 'CodeBlocks - Unix Makefiles'  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=clang++ -Dwithout-unittest=YES -DSSE=2;
Make = cd $(1); make -j$(2);
Clean = cd $(1); make clean;
all: $(WORK_DIR)/Makefile
	$(call Make, $(WORK_DIR), 5)
$(WORK_DIR)/Makefile:
	$(call CMake, $(WORK_DIR))
cmake:
	$(call CMake, $(WORK_DIR))
clean:
	$(call Clean, $(WORK_DIR))
tags:
	@ctags -R -f ./.git/ctags .
	@cscope -b -f ./.git/cscope.out
