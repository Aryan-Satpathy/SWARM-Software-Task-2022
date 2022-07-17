# Multilateration

As discussed in the task ps, you get signals from multiple stations.
From that you get distance of the bots from multiple stations.
Using that you must localise the bot.

<img 
    style="display: block; 
           margin-left: auto;
           margin-right: auto;
           width: 80%;"
    src="https://user-images.githubusercontent.com/86613790/179386888-8fa9b850-feb6-49b9-8fec-45d24ff91df9.png" 
    alt="Simulation">
</img>

## Requirements :
- Python 3.8 +
- OpenCV contrib
- Numpy

## Simulation Help
- Large green squares are stations which emit signals.
- Small blue square is the bot.
- Small green circle is the predicted pos.
- Red trail is that of the bot and green trail is that of the prediction.
- Text on top left are the distances from stations.

**Note:** Do not use trajectory properties for prediction. Because we might change trajectory during evaluation.
