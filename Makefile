all: prepare configure optional build install

prepare:
	chmod +x ./build.sh

docs: prepare
	./build.sh docs

configure: prepare
	./build.sh configure

optional: prepare
	./build.sh optional

build: prepare
	./build.sh build

install: build
	./build.sh install