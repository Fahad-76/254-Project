# tests below are scored
# remember to edit config.h accordingly before running this file!

./riscv -s -f ./code/ms2/input/R/R.input > ./code/ms2/out/R/R.trace
echo "diff ./code/ms2/ref/R/R.trace ./code/ms2/out/R/R.trace"
diff ./code/ms2/ref/R/R.trace ./code/ms2/out/R/R.trace

./riscv -s -f ./code/ms2/input/I/I.input > ./code/ms2/out/I/I.trace
echo "diff ./code/ms2/ref/I/I.trace ./code/ms2/out/I/I.trace"
diff ./code/ms2/ref/I/I.trace ./code/ms2/out/I/I.trace

./riscv -s -f ./code/ms2/input/LS/LS.input > ./code/ms2/out/LS/LS.trace
echo "diff ./code/ms2/ref/LS/LS.trace ./code/ms2/out/LS/LS.trace"
diff ./code/ms2/ref/LS/LS.trace ./code/ms2/out/LS/LS.trace

./riscv -s -e -f ./code/ms2/input/random.input > ./code/ms2/out/random.trace
echo "diff ./code/ms2/ref/random.trace ./code/ms2/out/random.trace"
diff ./code/ms2/ref/random.trace ./code/ms2/out/random.trace

./riscv -s -e -f ./code/ms2/input/multiply.input > ./code/ms2/out/multiply.trace
echo "diff ./code/ms2/ref/multiply.trace ./code/ms2/out/multiply.trace"
diff ./code/ms2/ref/multiply.trace ./code/ms2/out/multiply.trace

./riscv -s -e -f ./code/ms2/input/vec_xprod_tiny.input > ./code/ms2/out/vec_xprod_tiny.trace
echo "diff ./code/ms2/ref/vec_xprod_tiny.trace ./code/ms2/out/vec_xprod_tiny.trace"
diff ./code/ms2/ref/vec_xprod_tiny.trace ./code/ms2/out/vec_xprod_tiny.trace
