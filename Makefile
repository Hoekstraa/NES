tester:
	mkdir -p build
	gcc -O2 -lihct test/test_6502.c -o build/test_6502
	./build/test_6502
optimized:
	gcc -Ofast -flto -std=c99 -lihct test/test_6502.c -o build/test_6502
nestest:
	gcc -Wextra main.c && ./a.out log test/nestest.nes
