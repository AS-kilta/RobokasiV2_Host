//
// Created by lehdari on 13.10.2018.
//

#ifndef ROBOKASIV2_HOST_DHMATRIX_HPP
#define ROBOKASIV2_HOST_DHMATRIX_HPP


#include "MathTypes.hpp"


namespace kin {

    class DHMatrix {
    public:
        explicit DHMatrix(
            float d = 0.0f,
            float theta = 0.0f,
            float r = 0.0f, // also called a
            float alpha = 0.0f);

        /// Set joint angle
        void setTheta(float theta);

        operator const Mat4f&();

        const Mat4f& getMatrix();

    private:
        float   _d;
        float   _theta;
        float   _r;
        float   _alpha;

        Mat4f   _z; // joint matrix
        bool    _zDirty; // flag for joint matrix dirtiness
        Mat4f   _x; // link matrix
        bool    _xDirty; // flag for link matrix dirtiness
        Mat4f   _m; // transformation matrix (_z * _x)

        void update();
    };

} // namespace kin


#endif //ROBOKASIV2_HOST_DHMATRIX_HPP
