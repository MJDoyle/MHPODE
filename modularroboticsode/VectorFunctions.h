#ifndef VECTORFUNCTIONS_H
#define VECTORFUNCTIONS_H

#include <ode/ode.h>
#include <iostream>
#include <vector>
#include <math.h>

struct Vector3Di
{

private:

	int m_x, m_y, m_z;

public:

	//Constructor
	Vector3Di(dVector3 vec)
	{
		m_x = vec[0];
		m_y = vec[1];
		m_z = vec[2];
	}

	Vector3Di(int x, int y, int z)
	{
		m_x = x;
		m_y = y;
		m_z = z;
	}

	//Default contructor
	Vector3Di() {}

	//Getters
	const int Getx() const {return m_x;}
	const int Gety() const {return m_y;}
	const int Getz() const {return m_z;}

	//Setters
	void SetCoords(int x, int y, int z) {m_x = x; m_y = y; m_z = z;}

	Vector3Di operator+(Vector3Di other)
	{
		Vector3Di result;

		result.m_x = m_x + other.m_x;
		result.m_y = m_y + other.m_y;
		result.m_z = m_z + other.m_z;

		return result;
	}

	Vector3Di operator-(Vector3Di other)
	{
		Vector3Di result;

		result.m_x = m_x - other.m_x;
		result.m_y = m_y - other.m_y;
		result.m_z = m_z - other.m_z;

		return result;
	}

	Vector3Di CrossWith(Vector3Di other)
	{
		Vector3Di result;

		result.m_x = m_y * other.m_z - m_z * other.m_y;
		result.m_y = m_z * other.m_x - m_x * other.m_z;
		result.m_z = m_x * other.m_y - m_y * other.m_x;

		return result;
	}

	float DotWith(Vector3Di other)
	{
		return m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;
	}

	float ModulusSquared()
	{
		return m_x * m_x + m_y * m_y + m_z * m_z;
	}

	float Modulus()
	{
		return sqrt(float(m_x * m_x + m_y * m_y * m_z * m_z));
	}

	//Overloaded operators

	float operator[](std::size_t index) const
	{
		index = index % 3;

		if (index == 0)
		{
			return m_x;
		}

		else if (index == 1)
		{
			return m_y;
		}

		else if (index == 2)
		{
			return m_z;
		}
	}

	friend Vector3Di operator*(const float lhs, const Vector3Di rhs)
	{
		Vector3Di result;

		result.m_x = rhs.m_x * lhs;
		result.m_y = rhs.m_y * lhs;
		result.m_z = rhs.m_z * lhs;

		return result;
	}

	friend Vector3Di operator/(const Vector3Di lhs, const float rhs)
	{
		Vector3Di result;

		result.m_x = lhs.m_x / rhs;
		result.m_y = lhs.m_y / rhs;
		result.m_z = lhs.m_z / rhs;

		return result;
	}

	friend bool operator==(const Vector3Di lhs, const Vector3Di rhs)
	{
		if ((lhs.Getx() == rhs.Getx()) && (lhs.Gety() == rhs.Gety()) && (lhs.Getz() == rhs.Getz()))
		{
			return true;
		}

		else
		{
			return false;
		}
	}

	friend bool operator<(const Vector3Di lhs, const Vector3Di rhs)
	{
		if(lhs.Getx() < rhs.Getx())
		{
			return true;
		}

		else if(rhs.Getx() < lhs.Getx())
		{
			return false;
		}

		else if(lhs.Gety() < rhs.Gety())
		{
			return true;
		}

		else if(rhs.Gety() < lhs.Gety())
		{
			return false;
		}

		else if(lhs.Getz() < rhs.Getz())
		{
			return true;
		}

		else
		{
			return false;
		}
	}
};


struct Vector3Df
{

private:

	float m_x, m_y, m_z;

public:

	//Constructor
	Vector3Df(dVector3 vec)
	{
		m_x = vec[0];
		m_y = vec[1];
		m_z = vec[2];
	}

	Vector3Df(float x, float y, float z)
	{
		m_x = x;
		m_y = y;
		m_z = z;
	}

	//Default contructor
	Vector3Df() {}

	//Getters
	const float Getx() const {return m_x;}
	const float Gety() const {return m_y;}
	const float Getz() const {return m_z;}

	//Setters
	void SetCoords(float first, float second, float third, int start = 0) 
	{
		start = start % 3;

		if (start == 0)
		{
			m_x = first;
			m_y = second;
			m_z = third;
		}

		else if (start == 1)
		{
			m_x = third;
			m_y = first;
			m_z = second;
		}

		else if (start == 2)
		{
			m_x = second;
			m_y = third;
			m_z = first;
		}
	}

	void Normalize() 
	{
		float magnitude = sqrt(ModulusSquared());

		m_x = m_x / magnitude;
		m_y = m_y / magnitude;
		m_z = m_z / magnitude;
	}


	void Setx(float x) {m_x = x;}
	void Sety(float y) {m_y = y;}
	void Setz(float z) {m_z = z;}
	void SetElement(int i, float element) 
	{
		i = i % 3;

		if (i == 0)
		{
			m_x = element;
		}
		else if (i == 1)
		{
			m_y = element;
		}
		else if (i == 2)
		{
			m_z = element;
		}
	}

	//Overloaded operators

	Vector3Df operator-(Vector3Df other)
	{
		Vector3Df result;

		result.m_x = m_x - other.m_x;
		result.m_y = m_y - other.m_y;
		result.m_z = m_z - other.m_z;

		return result;
	}

	float operator[](std::size_t index) const
	{
		index = index % 3;

		if (index == 0)
		{
			return m_x;
		}

		else if (index == 1)
		{
			return m_y;
		}

		else if (index == 2)
		{
			return m_z;
		}
	}

	friend Vector3Df operator+(const Vector3Df lhs, const Vector3Df rhs)
	{
		Vector3Df result;

		result.m_x = lhs.m_x + rhs.m_x;
		result.m_y = lhs.m_y + rhs.m_y;
		result.m_z = lhs.m_z + rhs.m_z;

		return result;
	}

	friend Vector3Df operator*(const float lhs, const Vector3Df rhs)
	{
		Vector3Df result;

		result.m_x = rhs.m_x * lhs;
		result.m_y = rhs.m_y * lhs;
		result.m_z = rhs.m_z * lhs;

		return result;
	}

	friend Vector3Df operator/(const Vector3Df lhs, const float rhs)
	{
		Vector3Df result;

		result.m_x = lhs.m_x / rhs;
		result.m_y = lhs.m_y / rhs;
		result.m_z = lhs.m_z / rhs;

		return result;
	}

	friend bool operator==(const Vector3Df lhs, const Vector3Df rhs)
	{
		if ((lhs.Getx() == rhs.Getx()) && (lhs.Gety() == rhs.Gety()) && (lhs.Getz() == rhs.Getz()))
		{
			return true;
		}

		else
		{
			return false;
		}
	}

	friend bool operator<(const Vector3Df lhs, const Vector3Df rhs)
	{
		if(lhs.Getx() < rhs.Getx())
		{
			return true;
		}

		else if(rhs.Getx() < lhs.Getx())
		{
			return false;
		}

		else if(lhs.Gety() < rhs.Gety())
		{
			return true;
		}

		else if(rhs.Gety() < lhs.Gety())
		{
			return false;
		}

		else if(lhs.Getz() < rhs.Getz())
		{
			return true;
		}

		else
		{
			return false;
		}
	}

	Vector3Df CrossWith(Vector3Df other)
	{
		Vector3Df result;

		result.m_x = m_y * other.m_z - m_z * other.m_y;
		result.m_y = m_z * other.m_x - m_x * other.m_z;
		result.m_z = m_x * other.m_y - m_y * other.m_x;

		return result;
	}

	float DotWith(Vector3Df other)
	{
		return m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;
	}

	float ModulusSquared() const
	{
		return m_x * m_x + m_y * m_y + m_z * m_z;
	}

	float Modulus()
	{
		return sqrt(float(m_x * m_x + m_y * m_y + m_z * m_z));
	}
};

#endif


struct MatrixNMf
{
private:

	std::vector<std::vector<long double>> m_values;

	int m_rows;
	int m_columns;

public:

	//Default constructor
	MatrixNMf() {}

	//Constructor
	MatrixNMf(int rows, int columns)
	{
		m_rows = rows;
		m_columns = columns;

		for (int i = 0; i != m_rows; i ++)
		{
			std::vector<long double> vector;

			for (int j = 0; j != m_columns; j++)
			{
				if (i == j)
				{
					vector.push_back(1);
				}
				else
				{
					vector.push_back(0);
				}
			}

			m_values.push_back(vector);
		}
	}

	//Set rows and columns
	void SetSize(int rows, int columns)
	{
		m_rows = rows;
		m_columns = columns;

		for (int i = 0; i != m_rows; i ++)
		{
			std::vector<long double> vector;

			for (int j = 0; j != m_columns; j++)
			{
				if (i == j)
				{
					vector.push_back(1);
				}
				else
				{
					vector.push_back(0);
				}
			}

			m_values.push_back(vector);
		}
	}

	//Set a specific element
	void SetElement(int i, int j, long double value)
	{
		m_values[i][j] = value;
	}

	//Get an element
	long double GetElement(int i, int j)
	{
		return m_values[i][j];
	}

	//Set a column with the values
	void Set3Column(int j, Vector3Df vector)
	{
		for (int i = 0; i != 3; i ++)
		{
			m_values[i][j] = long double(vector[i]);
		}
	}

	std::vector<long double> GetRow(int i)
	{
		return m_values[i];
	}

	void SetRow(std::vector<long double> rowVector, int i)
	{
		if (rowVector.size() == m_columns)
		{
			m_values[i] = rowVector;
		}
	}


	std::vector<long double> GetColumn(int a)
	{
		std::vector<long double> column;

		for (int i = 0; i != m_rows; i ++)
		{
			for (int j = 0; j != m_columns; j ++)
			{
				if (j == a)
				{
					column.push_back(m_values[i][j]);
				}
			}
		}

		return column;

	}

	void SwapRows(int a, int b)
	{
		std::vector<long double> aRow = m_values[a];
		std::vector<long double> bRow = m_values[b];

		m_values[a] = bRow;
		m_values[b] = aRow;
	}

	void DivideRow(int i, long double a)
	{
		for (int j = 0; j != m_columns; j ++)
		{
			m_values[i][j] = m_values[i][j] / a;
		}
	}

	//Subtracts factor * a FROM b
	void SubtractMultipleOfRow(int a, int b, long double factor)
	{
		for (int j = 0; j != m_columns; j ++)
		{
			m_values[b][j] -= m_values[a][j] * factor;
		}

	}

	MatrixNMf GetTranspose()
	{
		MatrixNMf result(m_columns, m_rows);

		for (int i = 0; i != m_rows; i++)
		{
			for (int j = 0; j != m_columns; j++)
			{
				result.SetElement(j, i, m_values[i][j]);
			}
		}

		return result;
	}

	int GetNumRows()
	{
		return m_rows;
	}

	int GetNumColumns()
	{
		return m_columns;
	}

	//Find trace
	long double GetTrace()
	{
		//Check the matrix is square
		if (m_rows != m_columns)
		{
			std::cout << "Error - trying to find trace of non-square matrix" << std::endl;

			int err;
			std::cin >> err;
		}


		long double trace = 0;

		for (int i = 0; i != m_rows; i ++)
		{
			trace += m_values[i][i];
		}

		return trace;
	}

	//Find inverse by Gaussian elimination
	MatrixNMf GetInverse()
	{
		if (m_columns == m_rows)
		{
			//Set up augmented matrix
			MatrixNMf augmentedMatrix(m_rows, 2 * m_rows);

			for (int i = 0; i != m_rows; i ++)
			{
				for (int j = 0; j != m_rows; j ++)
				{
					augmentedMatrix.SetElement(i, j, m_values[i][j]);
				}

				for (int j = m_rows; j != m_rows * 2; j ++)
				{
					if ((j - m_rows) == i)
					{
						augmentedMatrix.SetElement(i, j, 1);
					}
					else
					{
						augmentedMatrix.SetElement(i, j, 0);
					}
				}
			}

			//Create row-echelon matrix
			//see http://en.wikipedia.org/wiki/Gaussian_elimination (at bottom)
			for (int k = 0; k != m_rows; k ++)
			{
				//Find k-th pivot
				int imax = 0;
				int currentMax = 0;
				for (int i = k; i != m_rows; i ++)
				{
					if (abs(augmentedMatrix.m_values[i][k]) > currentMax)
					{
						imax = i;
						currentMax = abs(augmentedMatrix.m_values[i][k]);
					}
				}

				//Check for singularity
				if (augmentedMatrix.m_values[imax][k] == 0)
				{
					std::cout << "Error - trying to find inverse of singular matrix" << std::endl;

					int err;
					std::cin >> err;
				}

				//Swap rows
				augmentedMatrix.SwapRows(imax, k);

				//Iterate through rows
				for (int i = k + 1; i != m_rows; i ++)
				{
					//Iterate through ALL columns
					for (int j = k + 1; j != 2 * m_rows; j++)
					{
						//Set elements
						augmentedMatrix.m_values[i][j] -= augmentedMatrix.m_values[k][j] * (augmentedMatrix.m_values[i][k] / augmentedMatrix.m_values[k][k]);
					}

					//Fill lower triangle with zeros
					augmentedMatrix.m_values[i][k] = 0;

				}
			}

			//Now we have matrix in row-echelon form

			//Iterate through rows starting from bottom
			for (int i = m_rows - 1; i != - 1; i--)
			{
				//Normalise the left-most non-zero number
				augmentedMatrix.DivideRow(i, augmentedMatrix.m_values[i][i]);

				//Iterate through all rows BELOW this row (starting from row just below this row)
				for (int i2 = i + 1; i2 != m_rows; i2 ++)
				{
					//subtract
					augmentedMatrix.SubtractMultipleOfRow(i2, i, augmentedMatrix.m_values[i][i2]);
				}
			}

			//Now get inverse matrix

			MatrixNMf inverseMatrix(m_rows, m_rows);

			for (int i = 0; i != m_rows; i++)
			{
				for (int j = 0; j != m_rows; j ++)
				{
					inverseMatrix.m_values[i][j] = augmentedMatrix.m_values[i][j + m_rows];
				}
			}

			return inverseMatrix;

		}

		else
		{
			std::cout << "Can't get inverse - matrix not square" << std::endl;

		}
	}

	Vector3Df DotWith3Vector(Vector3Df vector)
	{
		//Check that the matrix has the correct number of columns
		if (m_columns != 3)
		{
			std::cout << "Error - matrix dotting with 3vector, matrix wrong size" << std::endl;
			int err;
			std::cin >> err;
		}

		//You can actually dot a matrix with 2 rows (or 1) to a 3 vector, but lets just assumse its a 3x3 matrix for now
		long double values[3] = {0, 0, 0};

		for (int i = 0; i != 3; i ++)
		{
			for (int j = 0; j != 3; j ++)
			{
				values[i] += m_values[i][j] * long double(vector[j]);
			}

		}

		Vector3Df result(values[0], values[1], values[2]);

		return result;

	}

	MatrixNMf MatrixMultiply(MatrixNMf otherMatrix)
	{
		//Check that they're the correct sizes
		if (m_columns == otherMatrix.m_rows)
		{
			MatrixNMf result(m_rows, otherMatrix.m_columns);

			//Iterate through result and set values
			for (int i = 0; i != m_rows; i ++)
			{
				for (int j = 0; j != otherMatrix.m_columns; j++)
				{
					long double value = 0;
					for (int n = 0; n != m_columns; n ++)
					{
						value += m_values[i][n] * otherMatrix.m_values[n][j];
					}

					result.SetElement(i, j, value);
				}
			}

			return result;
		}

		else
		{
			std::cout << "Error - matrix multiplication. Matrices wrong size." << std::endl;
			int err;
			std::cin >> err;
		}

	}


	friend MatrixNMf operator*(const long double lhs, const MatrixNMf rhs)
	{
		static MatrixNMf result(rhs.m_rows, rhs.m_columns);

		for (int i = 0; i != rhs.m_rows; i ++)
		{
			for (int j = 0; j != rhs.m_columns; j ++)
			{
				result.m_values[i][j] = rhs.m_values[i][j] * lhs;
			}
		}

		return result;
	}


	friend MatrixNMf operator-(const MatrixNMf lhs, const MatrixNMf rhs)
	{
		//Check that the matrices are the same size
		if ((lhs.m_columns != rhs.m_columns) || (lhs.m_rows != rhs.m_rows))
		{
			std::cout << "Error - trying to subtract matrices of different sizes." << std::endl;
			int err;
			std::cin >> err;
		}

		MatrixNMf result(lhs.m_rows, lhs.m_columns);

		for (int i = 0; i != lhs.m_rows; i ++)
		{
			for (int j = 0; j != lhs.m_columns; j ++)
			{
				result.m_values[i][j] = lhs.m_values[i][j] - rhs.m_values[i][j];
			}
		}

		return result;
	}
};