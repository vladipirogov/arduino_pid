#ifndef PROPERTY_H
#define PROPERTY_H


template<typename T> class Property
{
public:
    Property() {}
    Property(const T & property) {this->property = property;}
    void set(const T & property) {this->property = property;}
    const float & get() {return this->property;}
private:
    T property;
};

#endif // PROPERTY_H
