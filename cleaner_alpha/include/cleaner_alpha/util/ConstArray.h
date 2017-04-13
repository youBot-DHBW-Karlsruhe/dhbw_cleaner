#ifndef CONSTARRAY_H
#define CONSTARRAY_H

namespace youbot_proxy {

namespace util {

template <class T, size_t MAX_LENGTH>
class ConstArrayFiller {
private:
    size_t length;

protected:
    void addElement(T* array, const T& element) {
        if(length >= MAX_LENGTH) {
            throw 0;
        }
        array[length] = element;
        length++;
    }

public:
    ConstArrayFiller(): length(0) {}
    ~ConstArrayFiller(){}

    virtual void fill(T* array) = 0;

};


template <class T, size_t MAX_LENGTH>
class ConstArray {
private:
    T array[MAX_LENGTH];

public:
    explicit ConstArray(ConstArrayFiller<T, MAX_LENGTH>& filler) {
        filler.fill(array);
    }

    T const& operator[](size_t i) const {
        assert(i >= 0 && i < MAX_LENGTH);
        return array[i];
    }

    size_t size() const {
        return MAX_LENGTH;
    }

    const T* getArray() const {
        return array;
    }
};

}

}

#endif // CONSTARRAY_H
