#include "rect.hpp"

/**
 * @brief Construct a new Rect object
 * 
 * @param xMin Left
 * @param yMin Top
 * @param xMax Right
 * @param yMax Bottom
 */
Rect::Rect(float xMin, float yMin, float xMax, float yMax) : top(yMin), left(xMin), bottom(yMax), right(xMax) {}

/**
 * @brief Retruns the degree to which another rectangle is contained within this rectangle.
 * 
 * @param other Other rectangle to check
 * @return float A number from 0 to 1 indicating how much the other rectangle is contained within this one. 0 meaning not at all and 1 meaning fully
 */
float Rect::overlapRatio(Rect *other) {
    // Fully outside
    if(right <= other->left || other->right <= left || top >= other->bottom || other->top >= bottom)
        return 0;

    float width, height;

    if(right >= other->right && left <= other->left)
        // On X axis, B is fully within A
        width = other->right - other->left;
    else if(other->right >= right && other->left <= left)
        // X axis A is fully within B
        width = right - left;
    else if(right >= other->right)
        width = other->right - left;
    else
        width = right - other->left;

    if(top <= other->top && bottom >= other->bottom)
        height = other->bottom - other->top;
    else if(other->top <= top && other->bottom >= bottom)
        height = bottom - top;
    else if(top <= other->top)
        height = bottom - other->top;
    else
        height = other->bottom - top;

    return (width * height) / ((other->right - other->left) * (other->bottom - other->top));
}
