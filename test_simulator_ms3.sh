#!/bin/bash

RED_BOLD='\033[1;31m'
YELLOW_BOLD='\033[1;33m'
GREEN_BOLD='\033[1;32m'
ORANGE_ITALIC='\033[38;5;214m\033[3m'
RESET='\033[0m'

# Function to handle SIGINT (Ctrl+C)
handle_sigint() {
    echo -e "${GREEN_BOLD}\nKudos!${RESET}"
    exit 0
}
trap handle_sigint SIGINT

# Function to run the first set of commands
run1() {
    echo -e "${RED_BOLD}Please make sure you are following the important note #1 in the milestone 3 description${RESET}"
       
   
    ./riscv -s -f -c   code/ms3/input/LS/LS.input > ./code/ms3/out/LS/LS.trace 
    echo "diff ./code/ms3/ref/LS/LS.trace ./code/ms3/out/LS/LS.trace"
    diff       ./code/ms3/ref/LS/LS.trace ./code/ms3/out/LS/LS.trace 

    ./riscv -s -f -c -e  ./code/ms3/input/multiply.input > ./code/ms3/out/multiply.trace 
    echo "diff ./code/ms3/ref/multiply.trace ./code/ms3/out/multiply.trace "
    diff       ./code/ms3/ref/multiply.trace ./code/ms3/out/multiply.trace  

    ./riscv -s -f -c -e  ./code/ms3/input/random.input > ./code/ms3/out/random.trace 
    echo "diff ./code/ms3/ref/random.trace ./code/ms3/out/random.trace"
    diff       ./code/ms3/ref/random.trace ./code/ms3/out/random.trace 

     ./riscv -s -f -c -e  ./code/ms3/input/testset_1.input > ./code/ms3/out/testset_1.trace 
    echo "diff ./code/ms3/ref/testset_1.trace  ./code/ms3/out/testset_1.trace"
    diff       ./code/ms3/ref/testset_1.trace  ./code/ms3/out/testset_1.trace 
    
}

# Function to run the second set of commands
run2() {
    echo -e "${YELLOW_BOLD}Please make sure you are following the important note #2 in the milestone 3 description${RESET}"
    echo -e "${ORANGE_ITALIC}If it takes more than 3 seconds, you probably have not followed the above message! Abort with Ctrl+C to save 1GB! ${RESET}"
    
    

    ./riscv -s -f -c -e  ./code/ms3/input/vec_xprod.input > ./code/ms3/out/vec_xprod.trace 
    echo "diff ./code/ms3/ref/vec_xprod.trace ./code/ms3/out/vec_xprod.trace"
    diff       ./code/ms3/ref/vec_xprod.trace ./code/ms3/out/vec_xprod.trace 
}


# Function to run the third set of commands
run3() {
    echo -e "${YELLOW_BOLD}Please make sure you are following the important note #3 in the milestone 3 description${RESET}"
    echo -e "${ORANGE_ITALIC}If it takes more than 5 seconds, you probably have not followed the above message! Abort with Ctrl+C to save 1GB! ${RESET}"
    
    

    ./riscv -s -f -e  ./code/ms3/input/vec_xprod.input > ./code/ms3/out/vec_xprod.nocache.trace 
    echo "diff ./code/ms3/ref/vec_xprod.nocache.trace ./code/ms3/out/vec_xprod.nocache.trace"
    diff       ./code/ms3/ref/vec_xprod.nocache.trace ./code/ms3/out/vec_xprod.nocache.trace 
}


# Check the first command-line argument and run the corresponding function
case $1 in
    cache_complete)
        run1
        ;;
    cache_summary)
        run2
        ;;
    no_cache)
        run3
        ;;
    *)
        echo "Usage: $0 {cache_complete|cache_summary|no_cache}"
        ;;
esac
