#include <iostream>
#include <string>

template<typename A>
class ClassA
{
public:
    ClassA(A number, int number_two = 2):
        m_number(number),
        m_number_two(number_two)
        {

        }
    A get_num(){
        return(m_number);
    }
    int get_num_two(){
        return(m_number_two);
    }
private:
    A m_number;
    int m_number_two;
};

template<typename A>
class ClassB
{
public:
    ClassB(ClassA<A> template_class):
        m_template_class(template_class)
        {

        }
    void print_nums(){
        std::cout << std::to_string(m_template_class.get_num()) << std::endl;
        std::cout << std::to_string(m_template_class.get_num_two()) << std::endl;
    }
    // ~ClassB();
private:
    ClassA<A> m_template_class;
};

class NormalClass
{
private:
    /* data */
public:
    NormalClass(/* args */);
};



int main(){
    int test_int = 1;
    ClassA<int> test_class_a(test_int);

    ClassB<int> test_class_b(test_class_a);
    test_class_b.print_nums();
}