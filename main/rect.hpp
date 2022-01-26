/**
 * @brief Structure for a rectangle using float coordinates * 
 */
struct Rect {
    float top, left, bottom, right;

    /**
     * @brief Construct a new Rect object
     * 
     * @param xMin Left
     * @param yMin Top
     * @param xMax Right
     * @param yMax Bottom
     */
    Rect(float xMin, float yMin, float xMax, float yMax);

    /**
     * @brief Retruns the degree to which another rectangle is contained within this rectangle.
     * 
     * @param other Other rectangle to check
     * @return float A number from 0 to 1 indicating how much the other rectangle is contained within this one. 0 meaning not at all and 1 meaning fully
     */
    float overlapRatio(Rect *other);
};