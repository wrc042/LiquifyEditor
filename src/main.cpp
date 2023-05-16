// #include "raylib.h"
// #define RAYGUI_IMPLEMENTATION
// #include "raygui.h"

// //------------------------------------------------------------------------------------
// // Program main entry point
// //------------------------------------------------------------------------------------
// int main(void) {
//     // Initialization
//     //--------------------------------------------------------------------------------------
//     const int screenWidth = 800;
//     const int screenHeight = 450;

//     InitWindow(screenWidth, screenHeight,
//                "raylib [textures] example - image loading");

//     // NOTE: Textures MUST be loaded after Window initialization (OpenGL
//     context
//     // is required)

//     Image image =
//         LoadImage("resources/raylib_logo.png"); // Loaded in CPU memory (RAM)
//     Texture2D texture = LoadTextureFromImage(
//         image);         // Image converted to texture, GPU memory (VRAM)
//     UnloadImage(image); // Once image has been converted to texture and
//     uploaded
//                         // to VRAM, it can be unloaded from RAM

//     SetTargetFPS(60); // Set our game to run at 60 frames-per-second
//     //---------------------------------------------------------------------------------------

//     // Main game loop
//     float value;
//     while (!WindowShouldClose()) // Detect window close button or ESC key
//     {
//         // Update
//         //----------------------------------------------------------------------------------
//         // TODO: Update your variables here
//         //----------------------------------------------------------------------------------

//         // Draw
//         //----------------------------------------------------------------------------------
//         BeginDrawing();

//         ClearBackground(RAYWHITE);
//         value = GuiSlider(Rectangle{96, 48, 216, 16},
//                           TextFormat("%0.2f", value), NULL, value,
//                           0.0f, 1.0f);

//         DrawTexture(texture, 0, 0, WHITE);

//         DrawText("this IS a texture loaded from an image!", 300, 370, 10,
//         GRAY);

//         DrawFPS(0, 0);

//         EndDrawing();
//         //----------------------------------------------------------------------------------
//     }

//     // De-Initialization
//     //--------------------------------------------------------------------------------------
//     UnloadTexture(texture); // Texture unloading

//     CloseWindow(); // Close window and OpenGL context
//     //--------------------------------------------------------------------------------------

//     return 0;
// }

#include "viewer/rl_euler_editor.hpp"

int main() {
    RLEulerEditor editor;
    editor.init("liquify editor");
    editor.reset_buffer();
    editor.run();
}
