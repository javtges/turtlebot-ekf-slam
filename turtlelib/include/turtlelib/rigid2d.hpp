#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>

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
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-8)
    {
        double diff = std::abs(d1-d2);
        if (diff > epsilon){
            return false;
        }
        else{
            return true;
        }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return (PI/180)*deg;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad * 180 / PI;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    static_assert(almost_equal(1, 1), "is_one failed");

    static_assert(almost_equal(deg2rad(90), PI/2), "deg2rad failed");

    static_assert(almost_equal(rad2deg(PI/2), 90), "rad2deg failed");

    static_assert(almost_equal(deg2rad(rad2deg(10)), 10), "deg2rad failed");



    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;


        /// \brief add two vectors together, returning their composition
        /// \param rhs - the right hand operand
        /// \return the addition of the two vectors
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief subtracts two vectors, returning their composition
        /// \param rhs - the right hand operand
        /// \return the difference of the two vectors
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief multiplies a vector by a scalar, returning their composition
        /// \param rhs - the right hand operand
        /// \return the product of the vector and scalar
        Vector2D & operator*=(const double & rhs);
    };

    /// \brief A 2-Dimensional Twist
    struct Twist2D
    {
        
        /// \brief the theta rotation 
        double thetadot = 0.0;
        
        /// \brief the x coordinate
        double xdot = 0.0;

        /// \breif the y coordinate
        double ydot = 0.0;

    };
    
    /// \brief normalizes an vector to a magnitude of 1
    /// \param v - the vector to be normalized
    /// \return the normalized vector
    Vector2D normalize(Vector2D v);

    /// \brief normalizes an angle to (-PI,PI] radians
    /// \param rad - the angle to be normalized
    /// \return the normalized angle
    double normalize_angle(double rad);

    /// \brief computes the dot product of two vectors
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the resulting double of the dot product
    double dot(Vector2D lhs, Vector2D rhs);

    /// \brief computes the magnitude of two vectors
    /// \param vec - the vector
    /// \return the magnitude of the vector
    double magnitude(Vector2D vec);

    /// /brief computes the angle between two vectors
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the double of the angle in radians
    double angle(Vector2D lhs, Vector2D rhs);

    /// \brief output a 2D Twist as [thetadot xdot ydot]
    /// os - output stream
    /// v - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & v);

    /// \brief input a 2D Twist as [thetadot xdot ydot]
    /// is - input stream
    /// v - the twist to set
    std::istream & operator>>(std::istream & is, Twist2D & v);


    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
        double T[3][3] = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}};
    
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
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;
        
        /// \brief apply a transformation to a Twist2D
        /// \param v - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation 
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
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief add two vectors together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the addition of the two vectors
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief subtracts two vectors, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the difference of the two vectors
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief multiply a vector by a scalar, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the resulting vector
    Vector2D operator*(Vector2D lhs, const double & rhs);

    /// \brief multiply a vector by a scalar, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the resulting vector
    Vector2D operator*(const double & lhs, Vector2D rhs);

    /// \brief integrate a twist over a timestep
    /// \param twist - the twist to be integrated
    /// \return the resulting transformation after the integration
    Transform2D integrate_twist(Twist2D twist);

}

#endif