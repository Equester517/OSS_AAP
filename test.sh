sudo rm -rf build

sudo -E cmake -B build .

sudo cmake --build build --target install -j $(nproc)

sudo -E build/install/para-exec.sh
