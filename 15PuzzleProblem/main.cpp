#include <iostream>
#include <fstream>
#include <string>

using namespace std;

#define size 4

bool isSolved = false;
int costs[16][16]; // matrix that stores the costs from each tile to each of the rest tiles 
int moveMatrix[16][4]; // matrix that stores the neighbours of each tile
unsigned int nodeCount = 0; // variable that stores the nodes explored for each search

// Function that creates a pattern database matrix for Manhattan distance heuristic
void makeMDMatrix()
{
	// Create a 15 x 15 matrix that stores the cost of each tile to each of the rest tiles
	for (int i = 0, sourceTile = 1; i < size * size; ++i, (++sourceTile) % (size * size))
	{
		for (int destinationTile = 0; destinationTile < size * size; ++destinationTile)
		{
			if (sourceTile == 0) // cost of source tile to source tile is 0
			{
				costs[sourceTile][destinationTile] = 0;
			}
			else
			{
				// compute Manhattan distance
				costs[sourceTile][destinationTile] = abs((i / size) - (destinationTile / size)) + abs((i % size) - (destinationTile % size));
			}
		}
	}
}

// Function that calculates the possibile moves each tile can have
void makeMovableMatrix()
{
	int moves[4][2] = { {-1, 0},  // left
						{1, 0},   // right
						{0, -1},  // up
						{0, 1} }; // down
	int board[4][4];

	// Create inital state board
	for (int i = 0; i < size; ++i)
	{
		for (int j = 0; j < size; ++j)
		{
			board[i][j] = j + i * size;
		}
	}

	int aux1, aux2, val;
	// Creates matrix that holds the neighbours (left, right, up, down) of the tiles
	// and puts -1 if it doesn't have a neighbour (when the tile is on the first and last rows and columns)
	for (int i = 0; i < size; ++i) // iterate through rows
	{
		for (int j = 0; j < size; ++j) // iterate through columns
		{
			for (int m = 0; m < 4; ++m) // iterate through moves[] array
			{
				aux2 = moves[m][0];
				aux1 = moves[m][1];
				if (j + aux2 < 0 || i + aux1 < 0 || j + aux2 >= size || i + aux1 >= size) // if you can't make a move (first and last rows and columns)
				{
					val = -1;
				}
				else
				{
					val = board[i + aux1][j + aux2]; // puts the number of the tile is in that place on the goal state board
				}
				moveMatrix[j + i * size][m] = val;
			}
		}
	}
}

// Linear conflict (adds 2 moves to the cost when two tiles are in their target rows or columns but inverted)
int getLinearConflictCost(int* row, int md, int n)
{
	if (n > 1)
	{
		if (n == 2)
		{
			if (row[0] > row[1])
				md += 2;
		}
		else if (n == 3)
		{
			if (row[0] > row[1] || row[0] > row[2])
				md += 2;
			if (row[1] > row[2])
				md += 2;
		}
		else if (n == 4)
		{
			if (row[0] > row[1] || row[0] > row[2] || row[0] > row[3])
				md += 2;
			if (row[1] > row[2] || row[1] > row[3])
				md += 2;
			if (row[2] > row[3])
				md += 2;
		}
	}
	return md;
}

// Compute distance
int getDistance(int* puzzle)
{
	int md = 0;
	int k, aux, x;
	int temp[4];
	
	// Calculates total cost
	for (int i = 0; i < size * size; ++i)
	{
		md = md + costs[puzzle[i]][i]; // cost[ puzzle[i] ] [i] will be the cost of the tile that is at puzzle[i]
	}

	// viewing the puzzle array as a matrix, meaning that putting the numbers in intervals of fours will get us the rows and columns of the matrix board
	// iterate through rows and see if linear conflict arsises
	for (int i = 0, x = 1; i < size; ++i, ++x) // i interates through "rows"
	{
		k = 0;
		for (int j = 0; j < size; ++j) // j iterates through "columns"
		{
			aux = puzzle[i * size + j];
			// check if it's on the first row (x = 1 => aux is in the interval 0-4 in the puzzle array, meaning that
			// on the puzzle board it would be on the first row), on the second row (x = 2 => interval 4-8 => second row) etc.
			if (aux <= size * x && aux > size * (x - 1))
			{
				temp[k++] = aux;
			}
		}
		md = getLinearConflictCost(temp, md, k); // compute linear conflict for this row
	}

	// iterate through columns and see if linear conflict arsises
	for (int i = 0, x = 1; i < size; ++i, ++x) // i iterates through "columns"
	{
		k = 0;
		for (int j = 0; j < size; ++j) // j iterates through "rows"
		{
			aux = puzzle[j * size + i];
			if (aux == x || aux == x + size || aux == x + size * 2 || aux == x + size * 3)
			{
				temp[k++] = aux;
			}
		}
		md = getLinearConflictCost(temp, md, k); // compute linear conflict for this column
	}
	return md;
}

// Function that finds and returns the index of the unoccupied position (blank)
int getBlankTile(int* puzzle)
{
	for (int i = 0; i < size * size; ++i)
	{
		if (puzzle[i] == 0)
		{
			return i;
		}
	}
}

// Function used to print the 15 Puzzle as matrix
void printPuzzle(int* puzzle)
{
	for (int i = 0; i < size; ++i)
	{
		for (int j = 0; j < size; ++j)
		{
			cout << (int)puzzle[i * size + j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}

// Recursive function used in IDAStar
int search(int depth, int maxDepth, int* puzzle, int lastMove, int blank)
{
	int h, f, min;
	nodeCount++;
	h = getDistance(puzzle);
	f = depth + h; // f = estimated cost of the cheapest path (root..node..goal)
				  // g = the cost to reach current node
				 // h = estimated cost of the cheapest path (node..goal)

	// greater cost encountered
	if (f > maxDepth)
	{
		return f;
	}

	// the puzzle is solved
	if (h == 0)
	{
		isSolved = true;
		return f;
	}

	min = 100;
	for (int move = 0; move < 4; ++move)
	{
		if (lastMove == -1 || (move + lastMove) % 4 != 1)
		{
			int newBlank = moveMatrix[blank][move];
			if (newBlank == -1)
				continue;

			puzzle[blank] = puzzle[newBlank];
			puzzle[newBlank] = 0;
			f = search(depth + 1, maxDepth, puzzle, move, newBlank); // recursive call with the next node as current node for depth search
			puzzle[newBlank] = puzzle[blank];
			puzzle[blank] = 0;

			if (f < min) // find the minimum of all f greater 
			{
				min = f;
			}

			if (isSolved)
			{
				return min;
			}
		}
	}
	return min;
}

// Iterative deepening A* function
void IDAStar(int* puzzle)
{
	time_t start, end; // for computation of solution time
	int h, depth;
	int lastMove = -1;
	int blank = getBlankTile(puzzle); // find the blank tile and store its index

	isSolved = false;
	unsigned int totalNodes = 0;

	h = getDistance(puzzle); // compute cost
	depth = h;
	start = clock();
	while (true) // loop forever
	{
		nodeCount = 0;
		depth = search(0, depth, puzzle, lastMove, blank);
		end = clock();
		totalNodes = totalNodes + nodeCount;

		if (isSolved)
		{
			cout << "Total number of nodes: " << totalNodes << "\nShortest path length: " << depth << " moves\nTime: " << (end - start) / 1000. << " milliseconds";
			return; // if the solution is found, exit function
		}
		h = depth;
	}
}

int main()
{
	int puzzle[16];

	for (int it = 0; it < 10; it++)
	{
		string fileName = "file";
		fileName.append(to_string(it));
		fileName.append(".txt");

		ifstream file(fileName);

		if (file.is_open())
		{
			for (int i = 0; i < 16; i++)
			{
				file >> puzzle[i];
			}
		}

		file.close();
		makeMDMatrix();
		makeMovableMatrix();

		cout << "Puzzle number " << it << ": " << endl << endl;
		printPuzzle(puzzle);
		IDAStar(puzzle);
		cout << endl << endl << "---------------------" << endl << endl;
	}

	return 0;
}