#include <iostream>

int initialization()
{
    //int a;         // default-initialization (no initializer)

    // Traditional initialization forms:
    // int b = 5;     // copy-initialization (initial value after equals sign)
    // int c(6);   // direct-initialization (initial value in parenthesis)

    // Modern initialization forms (preferred):
    // int d{ 7 };   // direct-list-initialization (initial value in braces)
    // int e{};      // value-initialization (empty braces)
    /*
    int a = 5, b = 6;          // copy-initialization
    int c(7), d(8);      // direct-initialization
    int e{ 9 }, f{ 10 };     // direct-list-initialization
    int i{}, j{};            // value-initialization
    */

    [[maybe_unused]] double pi{ 3.14159 };  // Don't complain if pi is unused
    [[maybe_unused]] double gravity{ 9.8 }; // Don't complain if gravity is unused
    [[maybe_unused]] double phi{ 1.61803 }; // Don't complain if phi is unused

    std::cout << pi << '\n';
    std::cout << phi << '\n';

    // The compiler will no longer warn about gravity not being used

    return 0;
}

int main()
{
    /*
    std::cout << "Hi!" << std::endl; // std::endl will cause the cursor to move to the next line
    std::cout << "My name is Alex." << std::endl;
	initialization();
    return 0;
    */
    /*
    std::cout << "Enter two numbers separated by a space: ";

    int x{}; // define variable x to hold user input (and value-initialize it)
    int y{}; // define variable y to hold user input (and value-initialize it)
    std::cin >> x >> y; // get two numbers and store in variable x and y respectively

    std::cout << "You entered " << x << " and " << y << '\n';

    return 0;
    */
    /*
    std::cout << "Enter two numbers: ";

    int x{};
    std::cin >> x;

    int y{};
    std::cin >> y;

    std::cout << "You entered " << x << " and " << y << '\n';

    return 0;
    */
    std::cout << "Enter a number: "; // ask user for a number
    int x{}; // define variable x to hold user input
    std::cin >> x; // get number from keyboard and store it in variable x
    std::cout << "You entered " << x << '\n';

    return 0;
}

