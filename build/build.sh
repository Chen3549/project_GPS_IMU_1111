cmake .. -B ./tmp
cmake --build tmp --target Positioning
cd tmp
make # build all
mv Positioning ../Positioning