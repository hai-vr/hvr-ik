using Microsoft.ML;
using Microsoft.ML.Data;

public class Program
{
    private readonly TrainMode _mode;

    private Program(TrainMode mode)
    {
        _mode = mode;
    }

    public static void Main()
    {
        new Program(TrainMode.Evaluate).Execute();
    }

    private void Execute()
    {
        var mlContext = new MLContext();

        // TODO: We should reprocess 000_tracker_data_record into a new dataset, because when the bend point is
        // collinear with the root and hand, it doesn't do a good job indicating the direction.
        // We should probably:
        // - expand the bend direction to be a tangent of the root--hand vector, and
        // - use the +Y direction of the bend quaternion instead when the points are collinear.
        
        var data = mlContext.Data.LoadFromTextFile<TrackerData>("Data/000_tracker_data_record.csv", hasHeader: false, separatorChar: ',');
        
        var dataCount = data.GetColumn<float>(nameof(TrackerData.A)).Count();
        Console.WriteLine($"Loaded {dataCount} rows of data");

        DataOperationsCatalog.TrainTestData trainTestData = default;
        if (_mode == TrainMode.Evaluate)
        {
            var preview = data.Preview(5);
            Console.WriteLine("First 5 rows of data:");
            foreach (var row in preview.RowView)
            {
                var values = row.Values.Select(v => v.Value?.ToString() ?? "null").ToArray();
                Console.WriteLine(string.Join(", ", values));
            }
            
            trainTestData = mlContext.Data.TrainTestSplit(data, testFraction: 0.2);
            data = trainTestData.TrainSet;
        }

        var pipelineA = mlContext.Transforms.Concatenate("Features", nameof(TrackerData.D), nameof(TrackerData.E), nameof(TrackerData.F))
            .Append(mlContext.Regression.Trainers.Sdca(nameof(TrackerData.A)));
        var pipelineB = mlContext.Transforms.Concatenate("Features", nameof(TrackerData.D), nameof(TrackerData.E), nameof(TrackerData.F))
            .Append(mlContext.Regression.Trainers.Sdca(nameof(TrackerData.B)));
        var pipelineC = mlContext.Transforms.Concatenate("Features", nameof(TrackerData.D), nameof(TrackerData.E), nameof(TrackerData.F))
            .Append(mlContext.Regression.Trainers.Sdca(nameof(TrackerData.C)));

        var modelA = pipelineA.Fit(data);
        var modelB = pipelineB.Fit(data);
        var modelC = pipelineC.Fit(data);

        if (_mode == TrainMode.TrainAllAndExport)
        {
            mlContext.Model.Save(modelA, data.Schema, "tracker_model_A.zip");
            mlContext.Model.Save(modelB, data.Schema, "tracker_model_B.zip");
            mlContext.Model.Save(modelC, data.Schema, "tracker_model_C.zip");
        }
        else
        {
            var predictionsA = modelA.Transform(trainTestData.TestSet);
            var metricsA = mlContext.Regression.Evaluate(predictionsA, labelColumnName: nameof(TrackerData.A));

            Console.WriteLine("Model A Metrics:");
            Console.WriteLine($"  R²: {metricsA.RSquared:F4}");
            Console.WriteLine($"  RMSE: {metricsA.RootMeanSquaredError:F4}");
            Console.WriteLine($"  MAE: {metricsA.MeanAbsoluteError:F4}");
        }
    }

    private enum TrainMode
    {
        TrainAllAndExport, Evaluate
    }
}

public class TrackerData
{
    [LoadColumn(0)] public float A { get; set; }
    [LoadColumn(1)] public float B { get; set; }
    [LoadColumn(2)] public float C { get; set; }
    [LoadColumn(3)] public float D { get; set; }
    [LoadColumn(4)] public float E { get; set; }
    [LoadColumn(5)] public float F { get; set; }
    // We ignore the quaternions for this attempt.
}

public class PredictionA { [ColumnName("Score")] public float PredictedA { get; set; } }
public class PredictionB { [ColumnName("Score")] public float PredictedB { get; set; } }
public class PredictionC { [ColumnName("Score")] public float PredictedC { get; set; } }
