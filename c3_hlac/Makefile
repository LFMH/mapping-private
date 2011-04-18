ARCH = $(shell uname -m)
LICENSE_URL = http://www.isi.imi.i.u-tokyo.ac.jp/~kanezaki/c3_hlac/pcl/license.txt
SAMPLE_DATA_URL = http://www.isi.imi.i.u-tokyo.ac.jp/~kanezaki/c3_hlac/pcl/sample.pcd
ifeq ($(ARCH), x86_64)
LIBRARY_URL = http://www.isi.imi.i.u-tokyo.ac.jp/~kanezaki/c3_hlac/pcl/x64/libc3_hlac.so
else
LIBRARY_URL = http://www.isi.imi.i.u-tokyo.ac.jp/~kanezaki/c3_hlac/pcl/x86/libc3_hlac.so
endif

$(shell mkdir -p $(shell rospack find c3_hlac)/lib)
ifeq ($(shell bash -c 'test -f $(shell rospack find c3_hlac)/lib/libc3_hlac.so && echo 1 || echo 0'), 0)
$(shell wget $(LIBRARY_URL) -P $(shell rospack find c3_hlac)/lib/)
endif
ifeq ($(shell bash -c 'test -f $(shell rospack find c3_hlac)/lib/license/license.txt && echo 1 || echo 0'), 0)
$(shell mkdir -p $(shell rospack find c3_hlac)/lib/license/)
$(shell wget $(LICENSE_URL) -P $(shell rospack find c3_hlac)/lib/license/)
endif

$(shell mkdir -p $(shell rospack find c3_hlac)/data)
ifeq ($(shell bash -c 'test -f $(shell rospack find c3_hlac)/data/sample.pcd && echo 1 || echo 0'), 0)
$(shell wget $(SAMPLE_DATA_URL) -P $(shell rospack find c3_hlac)/data/)
endif

include $(shell rospack find mk)/cmake.mk