# tests below are scored
# remember to enable DEBUG_REG_TRACE (check pipeline.h)
# and DISABLE ALL OTHER PRINTS before generating this output
./riscv -s ./code/ms1/input/R/R.input > ./code/ms1/out/R/R.trace
echo "diff ./code/ms1/ref/R/R.trace ./code/ms1/out/R/R.trace"
diff ./code/ms1/ref/R/R.trace ./code/ms1/out/R/R.trace

./riscv -s ./code/ms1/input/I/I.input > ./code/ms1/out/I/I.trace
echo "diff ./code/ms1/ref/I/I.trace ./code/ms1/out/I/I.trace"
diff ./code/ms1/ref/I/I.trace ./code/ms1/out/I/I.trace

./riscv -s ./code/ms1/input/LS/LS.input > ./code/ms1/out/LS/LS.trace
echo "diff ./code/ms1/ref/LS/LS.trace ./code/ms1/out/LS/LS.trace"
diff ./code/ms1/ref/LS/LS.trace ./code/ms1/out/LS/LS.trace

./riscv -s -e ./code/ms1/input/random.input > ./code/ms1/out/random.trace
echo "diff ./code/ms1/ref/random.trace ./code/ms1/out/random.trace"
diff ./code/ms1/ref/random.trace ./code/ms1/out/random.trace

./riscv -s -e ./code/ms1/input/multiply.input > ./code/ms1/out/multiply.trace
echo "diff ./code/ms1/ref/multiply.trace ./code/ms1/out/multiply.trace"
diff ./code/ms1/ref/multiply.trace ./code/ms1/out/multiply.trace

