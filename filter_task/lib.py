'''
Do not edit anything here.
'''

SIM_WIN_NAME = 'Multilateration Sim'

TxBuffer = []
RxBuffer = []

import typing
import time
import math
import random as rnd
import cv2
import numpy as np

coordinate = typing.Tuple[float]

v_wave = 200

class SignalsNotReady(Exception) :
    pass

class coordinate(tuple) : 
    def __add__(self, other) : 
        assert type(other) == coordinate
        result = coordinate([self[i] + other[i] for i in range(len(self))])
        return result
    def __sub__(self, other) : 
        assert type(other) == coordinate
        result = coordinate([self[i] - other[i] for i in range(len(self))])
        return result
    def __mul__(self, other) : 
        try : 
            assert type(other) == float
        except : 
            print(type(other))
            exit()
        result = coordinate([self[i] * other for i in range(len(self))])
        return result
    def __truediv__(self, other) : 
        assert type(other) in [int, float]
        if other == 0 : 
            print('div by 0')
            exit()
        return self * (1 / other)
    def mag(self) -> float : 
        return math.sqrt(sum([x * x for x in self]))

class Station : 
    idx = 0
    stations = []

    def __init__(self, pos : coordinate, frequency : float) :
        self.pos = pos
        self.frequency = frequency
        self.idx = Station.idx
        Station.idx += 1
        self.last_sent = -1
        Station.stations.append(self)
    def initialize(self) -> None : 
        if self.last_sent != -1 : 
            print('Station already initialized.')
            return
        self.last_sent = time.time() - 1 / self.frequency
        self.send()
    def send(self) -> None : 
        t = time.time()
        if t > self.last_sent + 1 / self.frequency : 
            self.last_sent = t
            TxBuffer.append((self.last_sent, self.idx))
        else : 
            return

class Reciever : 
    idx = 0
    def __init__(self, frequency : float) : 
        self.frequency = frequency
        self.idx = Reciever.idx
        Reciever.idx += 1
        self.last_recieved = -1
        self.times = [-1 for i in range(Station.idx)]
        self.distances = [None for i in range(Station.idx)]
    def initialize(self) -> None : 
        if self.last_recieved != -1 : 
            print('Reciever already initialized.')
            return
        self.last_recieved = -1
        self.recieve()
    def recieve(self) -> None : 
        t = time.time()
        if t > self.last_recieved + 1 / self.frequency : 
            self.last_recieved = t
            setonce = [False for i in range(len(Station.stations))]
            for data in RxBuffer : 
                if setonce[data[1]] : 
                    self.times[data[1]] = max(self.times[data[1]], data[0])
                else : 
                    setonce[data[1]] = True
                    self.times[data[1]] = data[0]
            self.distances = [(v_wave * (t - _t) if _t != -1 else None) for _t in self.times]
            # print(self.distances)
            RxBuffer.clear()
        else : 
            return

class Bot : 
    idx = 0
    def __init__(self, pos : coordinate, frequency : float) : 
        self.pos = pos
        self.idx = Bot.idx
        Bot.idx += 1
        self.frequency = frequency
        self.reciever = Reciever(frequency)
        self.trail = [Trail(self.pos, 5)]
        self.reciever.initialize()
        self.last_update = -1
        self.distances = [-1, -1, -1]
        self.predicted = coordinate(self.pos)
        self.prediction_trail = [Trail(self.predicted)]
        self._predict = None
    def move(self, velocity : coordinate) : 
        t = time.time()
        diff = t - self.last_update if not (self.last_update == -1) else 1 / self.frequency
        if t - self.last_update > 1 / self.frequency :
            self.last_update = t
        else : 
            return
        
        self.pos += velocity * diff 
        self.trail = [Trail(self.pos, 5)] + self.trail
    def gps(self) : 
        self.reciever.recieve()
        self.distances = self.reciever.distances
        self.predict()
        self.prediction_trail = [Trail(self.predicted)] + self.prediction_trail
    def predict(self) : 
        try : 
            self.predicted = self._predict([station.pos for station in Station.stations], self.distances)
        except (NotImplementedError, SignalsNotReady) :
            '''
            Placeholder prediction function
            '''
            count = 0
            for i in range(len(Station.stations)) : 
                station_ids = rnd.sample(range(len(Station.stations)), 3)
                
                s1, s2, s3 = Station.stations[station_ids[0]], Station.stations[station_ids[1]], Station.stations[station_ids[2]]
                '''
                Eq1 : 2(x1 - x2)x + 2(y1 - y2)y + x2^2 - x1^2 + y2^2 - y1^2 + d1^2 - d2^2
                Eq2 : 2(x2 - x3)x + 2(y2 - y3)y + x3^2 - x2^2 + y3^2 - y2^2 + d2^2 - d3^2
                '''

                if None in self.distances : 
                    continue

                a1 = 2 * (s1.pos[0] - s2.pos[0])
                b1 = 2 * (s1.pos[1] - s2.pos[1])
                c1 = (s2.pos[0] ** 2 - s1.pos[0] ** 2) + (s2.pos[1] ** 2 - s1.pos[1] ** 2) + (self.distances[station_ids[0]] ** 2 - self.distances[station_ids[1]] ** 2)

                a2 = 2 * (s2.pos[0] - s3.pos[0])
                b2 = 2 * (s2.pos[1] - s3.pos[1])
                c2 = (s3.pos[0] ** 2 - s2.pos[0] ** 2) + (s3.pos[1] ** 2 - s2.pos[1] ** 2) + (self.distances[station_ids[1]] ** 2 - self.distances[station_ids[2]] ** 2)

                A = np.array([[a1, b1], [a2, b2]])
                B = np.array([-c1, -c2])

                x = np.linalg.solve(A, B)
                self.predicted = self.predicted * float(count) + coordinate(x)
                count += 1
                self.predicted = self.predicted * (1 / count)

def transmit(bot : Bot) :
    t = time.time() 
    for data in TxBuffer : 
        idx, time_stamp = data[1], data[0]
        if t > time_stamp + (bot.pos - Station.stations[idx].pos).mag() / v_wave :
            RxBuffer.append(data)
            TxBuffer.remove(data)

class Trail : 

    max_lifetime = 1

    def __init__(self, pos : coordinate, lifetime = None) : 
        self.pos = pos
        self.lifetime = Trail.max_lifetime if lifetime is None else lifetime
        self.max_lifetime = Trail.max_lifetime if lifetime is None else lifetime

class Canvas : 

    station_color = (200, 255, 150)
    bot_color = (255, 200, 175)
    prediction_color = (150, 255, 150)
    trail_color = (100, 100, 255)
    prediction_trail_color = (150, 255, 150)

    def __init__(self, grid : coordinate, stations : typing.List[Station], bots : typing.List[Bot], frequency : float = 30) :
        self.grid = grid[ : : -1]
        self.stations = stations
        self.bots = bots
        self.trails = [bot.trail for bot in self.bots]
        self.predicted_trails = [bot.prediction_trail for bot in self.bots]
        self.frequency = frequency
        self.base = np.zeros((self.grid[0], self.grid[1], 3), dtype=np.uint8)
        self.store_base()
        self.img = self.base.copy()
        self.last_update = -1
        self.update()
    def store_base(self) : 
        for station in self.stations : 
            pos = coordinate(map(int, station.pos))
            cv2.rectangle(self.base, (pos - coordinate([3, 3])), (pos + coordinate([3, 3])), Canvas.station_color, -1)
    def update(self) : 
        
        t = time.time()

        if t - self.last_update > 1 / self.frequency : 
            self.last_update = time.time()
        else : 
            return

        self.img = self.base.copy()
        
        _img = self.img.copy()

        t = time.time()

        for j in range(len(self.trails)) :
            predicted_trail = self.predicted_trails[j] 
            deleted = 0
            predicted_trail[0].lifetime -= (t - self.last_update)
            for i in range(1, len(predicted_trail)) :
                i -= deleted
                particle = predicted_trail[i]
                alpha = math.pow(particle.lifetime / particle.max_lifetime, 2)
                pos = coordinate(map(int, particle.pos))
                _pos = coordinate(map(int, predicted_trail[i-1].pos))
                cv2.line(_img, _pos, pos, Canvas.prediction_trail_color, 1)
                try : 
                    self.img[min(_pos[1] - 1, pos[1] - 1, self.img.shape[0] - 1) : max(_pos[1], pos[1], 1) + 1, min(_pos[0] - 1, pos[0] - 1, self.img.shape[1] - 1) : max(_pos[0], pos[0], 1) + 1] = cv2.addWeighted(
                    self.img[min(_pos[1] - 1, pos[1] - 1, self.img.shape[0] - 1) : max(_pos[1], pos[1], 1) + 1, min(_pos[0] - 1, pos[0] - 1, self.img.shape[1] - 1) : max(_pos[0], pos[0], 1) + 1],
                    1 - alpha, 
                    _img[min(_pos[1] - 1, pos[1] - 1, self.img.shape[0] - 1) : max(_pos[1], pos[1], 1) + 1, min(_pos[0] - 1, pos[0] - 1, self.img.shape[1] - 1) : max(_pos[0], pos[0], 1) + 1],
                    alpha, 
                    0)
                except :
                    pass
                particle.lifetime -= 1 / self.frequency

            trail = self.trails[j]
            deleted = 0
            trail[0].lifetime -= (t - self.last_update)
            for i in range(1, len(trail)) :
                i -= deleted
                particle = trail[i]
                alpha = math.pow(particle.lifetime / particle.max_lifetime, 2)
                pos = coordinate(map(int, particle.pos))
                _pos = coordinate(map(int, trail[i-1].pos))
                cv2.line(_img, _pos, pos, Canvas.trail_color, 1)
                try : 
                    self.img[min(_pos[1] - 1, pos[1] - 1, self.img.shape[0] - 1) : max(_pos[1], pos[1], 1) + 1, min(_pos[0] - 1, pos[0] - 1, self.img.shape[1] - 1) : max(_pos[0], pos[0], 1) + 1] = cv2.addWeighted(
                    self.img[min(_pos[1] - 1, pos[1] - 1, self.img.shape[0] - 1) : max(_pos[1], pos[1], 1) + 1, min(_pos[0] - 1, pos[0] - 1, self.img.shape[1] - 1) : max(_pos[0], pos[0], 1) + 1],
                    1 - alpha, 
                    _img[min(_pos[1] - 1, pos[1] - 1, self.img.shape[0] - 1) : max(_pos[1], pos[1], 1) + 1, min(_pos[0] - 1, pos[0] - 1, self.img.shape[1] - 1) : max(_pos[0], pos[0], 1) + 1],
                    alpha, 
                    0)
                except :
                    pass
                particle.lifetime -= 1 / self.frequency

        for bot in self.bots : 
            pos = coordinate(map(int, bot.predicted))
            cv2.circle(self.img, pos, 2, Canvas.prediction_color, -1, cv2.LINE_AA)

            pos = coordinate(map(int, bot.pos))
            cv2.rectangle(self.img, (pos - coordinate([2, 2])), (pos + coordinate([2, 2])), Canvas.bot_color, -1)

        self.trails = [bot.trail for bot in self.bots]
        self.predicted_trails = [bot.prediction_trail for bot in self.bots]
        
        for bot in self.bots : 
            deleted = 0
            for i in range(len(bot.trail)) : 
                i -= deleted
                if bot.trail[i].lifetime < 0 : 
                    del bot.trail[i]
                    deleted += 1
            deleted = 0
            for i in range(len(bot.prediction_trail)) : 
                i -= deleted
                if bot.prediction_trail[i].lifetime < 0 : 
                    del bot.prediction_trail[i]
                    deleted += 1

        cv2.imshow(SIM_WIN_NAME, self.upscale())
        cv2.waitKey(1)
    def upscale(self, scale_factor = 1.9) : 
        img = cv2.resize(self.img, (0, 0), fx = scale_factor, fy = scale_factor, interpolation = cv2.INTER_NEAREST)

        size, baseline = cv2.getTextSize(str(self.bots[0].distances), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.putText(img, str(self.bots[0].distances), (0, size[1] + baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 100, 255), 2, cv2.LINE_AA)

        return img

def calliberate() : 
    return time.time()