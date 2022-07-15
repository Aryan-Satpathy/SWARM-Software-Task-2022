'''
We make 3 station points sending signals.
How do we simulate?
We can use a continuous function

'''

SIM_WIN_NAME = 'Kalman Filter Sim'

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
        assert type(other) == float
        result = coordinate([self[i] * other for i in range(len(self))])
        return result
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
    def move(self, velocity : coordinate) : 
        t = time.time()
        if t - self.last_update > 1 / self.frequency :
            self.last_update = t
        else : 
            return
        self.pos += velocity * (1 / self.frequency)
        self.trail = [Trail(self.pos, 5)] + self.trail
    def gps(self) : 
        self.reciever.recieve()
        self.distances = self.reciever.distances
        self.predict()
        self.prediction_trail = [Trail(self.predicted)] + self.prediction_trail
    def predict(self) : 

        station_ids = rnd.sample([0, 1, 2, 3], 3)
        station_ids = [0, 1, 2]

        s1, s2, s3 = Station.stations[station_ids[0]], Station.stations[station_ids[1]], Station.stations[station_ids[2]]
        '''
        Eq1 : 2(x1 - x2)x + 2(y1 - y2)y + x2^2 - x1^2 + y2^2 - y1^2 + d1^2 - d2^2
        Eq2 : 2(x2 - x3)x + 2(y2 - y3)y + x3^2 - x2^2 + y3^2 - y2^2 + d2^2 - d3^2
        '''

        if None in self.distances : 
            return

        a1 = 2 * (s1.pos[0] - s2.pos[0])
        b1 = 2 * (s1.pos[1] - s2.pos[1])
        c1 = (s2.pos[0] ** 2 - s1.pos[0] ** 2) + (s2.pos[1] ** 2 - s1.pos[1] ** 2) + (self.distances[station_ids[0]] ** 2 - self.distances[station_ids[1]] ** 2)

        a2 = 2 * (s2.pos[0] - s3.pos[0])
        b2 = 2 * (s2.pos[1] - s3.pos[1])
        c2 = (s3.pos[0] ** 2 - s2.pos[0] ** 2) + (s3.pos[1] ** 2 - s2.pos[1] ** 2) + (self.distances[station_ids[1]] ** 2 - self.distances[station_ids[2]] ** 2)

        A = np.array([[a1, b1], [a2, b2]])
        B = np.array([-c1, -c2])

        x = np.linalg.solve(A, B)
        self.predicted = coordinate(x)
def transmit(bot : Bot) :
    t = time.time() 
    for data in TxBuffer : 
        idx, time_stamp = data[1], data[0]
        if t > time_stamp + (bot.pos - Station.stations[idx].pos).mag() / v_wave :
            RxBuffer.append(data)
            TxBuffer.remove(data)

def main() : 

    grid = coordinate([1000, 500])
    max_vel = coordinate([15, 15])

    num_stations = 4
    for i in range(num_stations) : 
        Station(coordinate([rnd.random() * grid[0], rnd.random() * grid[1]]), 600)

    bot = Bot(coordinate([rnd.random() * grid[0] / 2 +  grid[0] / 2, rnd.random() * grid[1] / 2 +  grid[1] / 2]), 20)

    for station in Station.stations : 
        station.initialize()

    canvas = Canvas(grid, Station.stations, [bot], 10)

    key = 0

    velocities = [coordinate([x, y]) for x in [-max_vel[0], 0, max_vel[0]] for y in [-max_vel[1], 0, max_vel[1]]]
    id_x = rnd.choice(range(len(velocities)))

    while (key & 0xff) not in [27, 81, 113]: 

        for station in Station.stations : 
            station.send()

        transmit(bot)

        # velocity = coordinate([2 * (rnd.random() - 0.5) * max_vel[0], 2 * (rnd.random() - 0.5) * max_vel[1]])
        if rnd.random() > 0.9 : 
            id_x = rnd.choice(range(len(velocities)))
        
        velocity = velocities[id_x]
        bot.move(velocity)

        bot.gps()

        canvas.update()

        key = cv2.waitKey(1)

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
            # cv2.rectangle(self.base, (pos - coordinate([3, 3]))[ : : -1], (pos + coordinate([3, 3]))[ : : -1], Canvas.station_color, -1)
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
            # print(len(trail))
            predicted_trail[0].lifetime -= (t - self.last_update)
            for i in range(1, len(predicted_trail)) :
                i -= deleted
                particle = predicted_trail[i]
                alpha = math.pow(particle.lifetime / particle.max_lifetime, 2)
                pos = coordinate(map(int, particle.pos))
                _pos = coordinate(map(int, predicted_trail[i-1].pos))
                cv2.line(_img, _pos, pos, Canvas.prediction_trail_color, 1)
                # cv2.line(_img, _pos[ : : -1], pos[ : : -1], Canvas.trail_color, 1, cv2.LINE_AA)
                # cv2.circle(_img, pos, 1, Canvas.trail_color, -1)
                # cv2.imshow('debuug', self.img[min(_pos[0], pos[0]) : max(_pos[0], pos[0]) + 1, min(_pos[1], pos[1]) : max(_pos[1], pos[1]) + 1])
                # cv2.waitKey(0)
                # cv2.imshow('debuug', _img[min(_pos[0], pos[0]) : max(_pos[0], pos[0]) + 1, min(_pos[1], pos[1]) : max(_pos[1], pos[1]) + 1])
                # cv2.waitKey(0)
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
            # print(len(trail))
            trail[0].lifetime -= (t - self.last_update)
            for i in range(1, len(trail)) :
                i -= deleted
                particle = trail[i]
                alpha = math.pow(particle.lifetime / particle.max_lifetime, 2)
                pos = coordinate(map(int, particle.pos))
                _pos = coordinate(map(int, trail[i-1].pos))
                cv2.line(_img, _pos, pos, Canvas.trail_color, 1)
                # cv2.line(_img, _pos[ : : -1], pos[ : : -1], Canvas.trail_color, 1, cv2.LINE_AA)
                # cv2.circle(_img, pos, 1, Canvas.trail_color, -1)
                # cv2.imshow('debuug', self.img[min(_pos[0], pos[0]) : max(_pos[0], pos[0]) + 1, min(_pos[1], pos[1]) : max(_pos[1], pos[1]) + 1])
                # cv2.waitKey(0)
                # cv2.imshow('debuug', _img[min(_pos[0], pos[0]) : max(_pos[0], pos[0]) + 1, min(_pos[1], pos[1]) : max(_pos[1], pos[1]) + 1])
                # cv2.waitKey(0)
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
            # cv2.rectangle(self.img, (pos - coordinate([2, 2]))[ : : -1], (pos + coordinate([2, 2]))[ : : -1], Canvas.bot_color, -1)
            cv2.circle(self.img, pos, 2, Canvas.prediction_color, -1, cv2.LINE_AA)

            pos = coordinate(map(int, bot.pos))
            # cv2.rectangle(self.img, (pos - coordinate([2, 2]))[ : : -1], (pos + coordinate([2, 2]))[ : : -1], Canvas.bot_color, -1)
            cv2.rectangle(self.img, (pos - coordinate([2, 2])), (pos + coordinate([2, 2])), Canvas.bot_color, -1)

        self.trails = [bot.trail for bot in self.bots]
        self.predicted_trails = [bot.prediction_trail for bot in self.bots]
        
        # _img = cv2.copyTo(self.img, np.ones(self.img.shape[ : -1]))
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

        # self.img = _img
        cv2.imshow(SIM_WIN_NAME, self.upscale())
        cv2.waitKey(1)
    def upscale(self, scale_factor = 1.9) : 
        # img = np.ones()
        img = cv2.resize(self.img, (0, 0), fx = scale_factor, fy = scale_factor, interpolation = cv2.INTER_NEAREST)
        # img = self.base.copy()

        size, baseline = cv2.getTextSize(str(self.bots[0].distances), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.putText(img, str(self.bots[0].distances), (0, size[1] + baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 100, 255), 2, cv2.LINE_AA)

        return img


if __name__ == '__main__' : 
    main()