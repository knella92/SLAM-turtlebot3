#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
       double diff{d1-d2};
        if(std::abs(diff) < epsilon){
            return true;
        }
        else{
            return false;
        }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return (deg/360.0*2.0*PI);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return (rad/(2.0*PI)*360.0);
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(3.0+1e-13,3.0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(360), 2*PI), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    static_assert(almost_equal(rad2deg(2*PI), 360.0), "rad2deg failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
    static_assert(almost_equal(deg2rad(rad2deg(45)), 45), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief compute addition of two 2D vectors and store result in this object
        /// \param rhs - vector to add to original
        /// \return sum of the two vectors
        Vector2D& operator+=(const Vector2D& rhs);

        /// \brief compute subtraction of two 2D vectors and store result in this object
        /// \param rhs - vector to subtract from the original
        /// \return difference of the two vectors
        Vector2D& operator-=(const Vector2D& rhs);

    };

    /// \brief compute the dot product of two 2D vectors
    /// \param vec1 - first vector
    /// \param vec2 - second vector
    /// \return - dot product with double point precision
    double dot(Vector2D& vec1, Vector2D& vec2);

    /// \brief compute the magnitude of a 2D vector
    /// \param vec - vector
    /// \return - magnitude of the vector
    double magnitude(Vector2D& vec);

    /// \brief compute the angle between two 2D vectors
    /// \param vec1 - first vector
    /// \param vec2 - second vector
    /// \return - angle between the vectors
    double angle(Vector2D& vec1, Vector2D& vec2);

    /// \brief output the normalized 2D vector of the input parameter
    /// \param v - 2D vector input
    /// \return normalized vector
    Vector2D normalize(Vector2D v);

    /// \brief adds two vectors together, returning their sum
    /// \param lhs - left hand side of operator
    /// \param rhs - right hand side of operator
    /// \return - the vector sum
    Vector2D operator+(Vector2D lhs, const Vector2D rhs);

    /// \brief gives difference between two vectors together
    /// \param lhs - left hand side of operator
    /// \param rhs - right hand side of operator
    /// \return - the vector difference
    Vector2D operator-(Vector2D lhs, const Vector2D rhs);

    /// \brief gives product of left scalar multiplication of 2D vector
    /// \param lhs - scalar value (left side)
    /// \param rhs - 2D vector to be multiplied (right side)
    /// \return - the final 2D vector
    Vector2D operator*(double lhs, const Vector2D rhs);

    /// \brief gives product of right scalar multiplication of 2D vector
    /// \param lhs - 2D vector (left side)
    /// \param rhs - scalar value to multiply by (right side)
    /// \return - the final 2D vector
    Vector2D operator*(Vector2D lhs, const double rhs);



    /// \brief output the normalized angle of the input paramater
    /// \param rad - angular rotation input
    /// \return normalized angular rotation
    double normalize_angle(double rad);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user types
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin) and processing stops.
    ///
    /// We have lower level control however.
    /// std::peek() looks at the next unprocessed character in the buffer without removing it
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// std::get() removes the next unprocessed character from the buffer.
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/get
    /// When you call std::peek() it will wait for there to be at least one character in the buffer (e.g., the user types a character)
    /// HINT: this function can be written in under 20 lines and uses only std::peek(), std::get(), istream::operator>>() and a little logic
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief A twist in two dimensions 
    struct Twist2D
    {
        /// \brief the angular velocity
        double w;
        
        /// \brief the 2D vector of the translational velocity
        Vector2D v;

        /// \brief Create an empty twist
        Twist2D();

        /// \brief Create a twist with a rotational and translational
        /// component
        /// \param w - the angular velocity input
        /// \param v - the 2D vector of the translational velocity input
        Twist2D(double w, Vector2D v);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

    };

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:

        /// \brief Transformation matrix element [0][0]
        double t00;

        /// \brief Transformation matrix element [0][1]
        double t01;

        /// \brief Transformation matrix element [0][2]
        double t02;

        /// \brief Transformation matrix element [1][0]
        double t10;

        /// \brief Transformation matrix element [1][1]
        double t11;

        /// \brief Transformation matrix element [1][2]
        double t12;

        /// \brief Transformation matrix element [2][0]
        double t20;

        /// \brief Transformation matrix element [2][1]
        double t21;

        /// \brief Transformation matrix element [2][2]
        double t22;

    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(Vector2D trans, double rot);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;


        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

        /// \brief convert a twist V to a different reference frame
        /// \param V - the twist to transform
        /// \return a twist in the new reference frame
        Twist2D operator()(Twist2D V) const;

    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    /// For example:
    /// 90 2 3
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief should print a human readable version of the Twist:
    /// An example output:
    /// [1 3 5]
    /// \param os - an output stream
    /// \param tw - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

    /// \brief Read a twist from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (w x y) separated by spaces
    /// For example:
    /// 1 2 3
    /// \param is - input stream
    /// \param tw - input twist
    std::istream & operator>>(std::istream & is, Twist2D & tw);

    /// \brief Computes the transformation corresponding to a rigid body
    /// following a constant twist (in its original body frame) for one
    /// time unit
    /// \param Vb - constant twist within original body frame of rigid body
    /// \return transformation corresponding new body frame from original body frame
    Transform2D integrate_twist(Twist2D Vb);


}

#endif
