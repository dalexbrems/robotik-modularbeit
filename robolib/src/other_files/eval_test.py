from numpy import array, all, fromstring
import ast

# arr_1 = array(['1','2','3'])

# arr_string = repr(arr_1)
# print(arr_string)
#
# arr_2 = eval(arr_string)
#
# print(arr_2.__repr__())
#
# print(all(arr_1 == arr_2))

teststring = '[-0.00000255 -0.00000001  0.00000521  0.00000003 -0.00001278 -0.        ]'

arr = ast.literal_eval(teststring)
print(arr)
