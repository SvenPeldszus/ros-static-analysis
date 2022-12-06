import json
from classes import class_itself

class Class_complex(class_itself):
    field_one: str = None
    
    def __init__(self, field_one: str):
        self.field_one = field_one
        
    def func1(self):
        self.func2(self.field_one)
    
    def func2(self, str_to_print):
        print(f"Your output: {str_to_print}")


def main():
    class_complex = Class_complex("test")
    class_complex.func1()
    
if __name__ == "__main__":
    main()
        
def func1(self):
    self.func2(self.field_one)