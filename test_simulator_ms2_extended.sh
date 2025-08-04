# tests below are scored
# remember to update config.h accordingly before running this!

# full version of vec_xprod only contains the stats of the program, not the entire reg-trace
./riscv -s -e -f ./code/ms2/input/vec_xprod.input > ./code/ms2/out/vec_xprod.trace
echo "diff ./code/ms2/ref/vec_xprod.trace ./code/ms2/out/vec_xprod.trace"
diff ./code/ms2/ref/vec_xprod.trace ./code/ms2/out/vec_xprod.trace
