import numpy as np
def corr_matrix_estimate(X, imp="mem_eff"):
    """
        Estimates the spatial correlation matrix with sample averaging    
    
    Implementation notes:
    --------------------
        Two different implementation exist for this function call. One of them use a for loop to iterate through the
        signal samples while the other use a direct matrix product from numpy. The latter consumes more memory
        (as all the received coherent multichannel samples must be available at the same time)
        but much faster for large arrays. The implementation can be selected using the "imp" function parameter.
        Set imp="mem_eff" to use the memory efficient implementation with a for loop or set to "fast" in order to use
        the faster direct matrix product implementation.
    
        
    Parameters:
    -----------
        :param X : Received multichannel signal matrix from the antenna array.         
        :param imp: Selects the implementation method. Valid values are "mem_eff" and "fast". The default value is "mem_eff".
        :type X: N x M complex numpy array N is the number of samples, M is the number of antenna elements.
        :type imp: string
            
    Return values:
    -------------
    
        :return R : Estimated spatial correlation matrix
        :rtype R: M x M complex numpy array
        
        :return -1 : When unidentified implementation method was specified
    """      
    N = np.size(X, 0)
    M = np.size(X, 1)
    R = np.zeros((M, M), dtype=complex)    
    
    # --input check--
    if N < M:
        print("WARNING: Number of antenna elements is greather than the number of time samples")
        print("WARNING: You may flipped the input matrix")
    
    # --calculation--
    if imp == "mem_eff":            
        for n in range(N):
            R += np.outer(X[n, :], np.conjugate(X[n, :]))
    elif imp == "fast":
            X = X.T 
            R = np.dot(X, X.conj().T)
    else:
        print("ERROR: Unidentified implementation method")
        print("ERROR: No output is generated")
        return -1
        
    R = np.divide(R, N)
    return R

