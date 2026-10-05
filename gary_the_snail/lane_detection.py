import cv2
import numpy as np
import random

def crop_half(img):
    return img[img.shape[0]//2:, :] 

def detect_lines(img, threshold1=50, threshold2=150, aperture_size=3, minLineLength=100, maxLineGap=10):
    grayscale_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grayscale_img, threshold1, threshold2, apertureSize=aperture_size)

    lines = cv2.HoughLinesP(
        edges,
        rho=aperture_size,
        theta=np.pi/180,
        threshold=100,
        minLineLength=minLineLength,
        maxLineGap=maxLineGap
    )

    if lines is None:
        return []

    return lines

def draw_lines(img, lines, color=(0, 255, 0)):
    if lines is None:
        return img

    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), color, 2)
    
    return img

def get_slopes_intercepts(lines):
    """
    Returns slopes and horizontal intercepts with the bottom of an image for a set of images

    Params
    - lines: list of lines

    Ret
    - (slopes, intercepts): tuple of slopes then intercepts
    """
    slopes = []
    intercepts = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        slopes.append((y2 - y1) / (x2 - x1))

        intercepts.append((-y1) / slopes[-1] + x1)

    return (slopes, intercepts)

def detect_lanes(lines):
    lanes = []

    slopes, intercepts = get_slopes_intercepts(lines)
    
    filtered_lines = set()
    seen = [False] * len(lines)
    for i in range(len(lines)):
        if seen[i]:
            continue
        
        for j in range(i+1, len(lines)):
            if seen[j]:
                continue

            int_sim = abs(intercepts[i] - intercepts[j]) <= 25
            ang_sim = abs(np.arctan(slopes[i]) - np.arctan(slopes[j])) <= 0.05

            if int_sim and ang_sim:
                seen[j] = True
        
        filtered_lines.add(i)

    filtered_lines = list(filtered_lines)

    # x = [lines[fl] for fl in filtered_lines]
    # print(x)
    # print([intercepts[fl] for fl in filtered_lines])
    # print([np.arctan(slopes[fl]) for fl in filtered_lines])

    for i in range(len(filtered_lines)):
        for j in range(i+1, len(filtered_lines)):
            angle1 = np.arctan(slopes[filtered_lines[i]])
            angle2 = np.arctan(slopes[filtered_lines[j]])

            ang_sim = abs(angle1 - angle2) <= 0.2

            if ang_sim:
                lanes.append([lines[filtered_lines[i]], lines[filtered_lines[j]]])
                
    return lanes

def draw_lanes(img, lanes):
    for lane in lanes:
        color = (random.randint(0, 256), random.randint(0, 256), random.randint(0, 256))

        x1, y1, x2, y2 = lane[0][0]
        cv2.line(img, (x1, y1), (x2, y2), color, 10)

        x1, y1, x2, y2 = lane[1][0]
        cv2.line(img, (x1, y1), (x2, y2), color, 10)
    
    return img