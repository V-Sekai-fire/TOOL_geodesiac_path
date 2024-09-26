export URL := "https://github.com/godotengine/godot/releases/download/4.3-stable/Godot_v4.3-stable_win64.exe.zip"
export FILENAME := "Godot_v4.3-stable_win64.exe.zip"
export EXTRACTED_FILE := "Godot_v4.3-stable_win64.exe"

build:
    mkdir build -p
    cd build && cmake .. -G "Ninja" -DCMAKE_BUILD_TYPE=Release
    cmake --build build --parallel

run:
    @just build
    # ./bin/flip_geodesics mesh.obj

clean:
    rm -rf build
