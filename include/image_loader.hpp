#pragma once

// Simple helper function to load an image into a OpenGL texture with common settings
bool LoadTextureFromFile(const std::string &filename, unsigned int &img_texture, int &img_width, int &img_height);
