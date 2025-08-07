#include<format>
#include<iostream>
#include<memory>
using namespace std;

struct m_Ptr {
	string name{};
	int num{};
	void print() {
		cout << format("my name is {}", name) << endl;
	}
};

struct ptrTrans {
	void getPtr(const unique_ptr<m_Ptr>& Ptr) {
		if (!m_ptr) {
			m_ptr = make_unique<m_Ptr>();
		}
		*m_ptr = *Ptr; // 初始化时 m_ptr 是空指针，不能解引用
		// m_ptr = std::move(Ptr); // unique_ptr<m_Ptr>& Ptr
	}
	unique_ptr<m_Ptr> m_ptr{}; // 空指针
};

int main()
{
	auto m_ptr =  make_unique<m_Ptr>();
	m_ptr->name = "first ptr";
	ptrTrans a;
	a.getPtr(m_ptr);
	cout << a.m_ptr->name << endl;

}