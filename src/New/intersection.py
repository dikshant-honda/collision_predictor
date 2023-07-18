from math import sqrt, acos, floor, sin

# Function to return area of intersection
def intersectionArea(X1, Y1, R1, X2, Y2, R2):

    Pi = 3.14
    
    # Calculate the euclidean distance
    # between the two points
    d = sqrt(((X2 - X1) * (X2 - X1)) + ((Y2 - Y1) * (Y2 - Y1)))

    if (d > R1 + R2) :
        ans = 0

    elif (d <= (R1 - R2) and R1 >= R2) :
        ans = floor(Pi * R2 * R2)

    elif (d <= (R2 - R1) and R2 >= R1) :
        ans = floor(Pi * R1 * R1)

    else :
        alpha = acos(((R1 * R1) + (d * d) - (R2 * R2)) / (2 * R1 * d)) * 2
        beta = acos(((R2 * R2) + (d * d) - (R1 * R1)) / (2 * R2 * d)) * 2
        
        a1 = (0.5 * beta * R2 * R2 ) - (0.5 * R2 * R2 * sin(beta))
        a2 = (0.5 * alpha * R1 * R1) - (0.5 * R1 * R1 * sin(alpha))
        ans = floor(a1 + a2)

    return ans

# Driver Code
if __name__ == "__main__" :

    X1 = 0; Y1 = 0; R1 = 4
    X2 = 6; Y2 = 0; R2 = 4

    print(intersectionArea(X1, Y1, R1, X2, Y2, R2))
    