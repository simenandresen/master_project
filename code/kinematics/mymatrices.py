#!/usr/bin/python

from sympy import *
from sympy import var
from sympy import symbols, BlockMatrix, Matrix

def mymat(n,m,letter='r'):
	M = Matrix(n, m, lambda i,j:var(letter+'_%d%d' % (i+1,j+1)))
	return M


a=mymat(3,4)

print latex(a)


