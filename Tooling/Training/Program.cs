// Copyright 2025 Haï~ (@vr_hai github.com/hai-vr)
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//    http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System.Globalization;
using System.Numerics;
using Microsoft.ML;
using Microsoft.ML.Data;
using Tooling;

public class Program
{
    private readonly ExecutionMode _mode;

    private Program(ExecutionMode mode)
    {
        _mode = mode;
    }

    public static void Main()
    {
        new Program(ExecutionMode.GenerateLookupTable).Execute();
    }

    private void Execute()
    {
        var mlContext = new MLContext();

        // TODO: We should reprocess 000_tracker_data_record into a new dataset, because when the bend point is
        // collinear with the root and hand, it doesn't do a good job indicating the direction.
        // We should probably:
        // - expand the bend direction to be a tangent of the root--hand vector, and
        // - use the +Y direction of the bend quaternion instead when the points are collinear.

        if (_mode != ExecutionMode.GenerateLookupTable)
        {
            TrainingMode(mlContext);
        }
        else
        {
            var modelA = mlContext.Model.Load("tracker_model_A.zip", out _);
            var modelB = mlContext.Model.Load("tracker_model_B.zip", out _);
            var modelC = mlContext.Model.Load("tracker_model_C.zip", out _);
    
            var engineA = mlContext.Model.CreatePredictionEngine<TrackerData, PredictionA>(modelA);
            var engineB = mlContext.Model.CreatePredictionEngine<TrackerData, PredictionB>(modelB);
            var engineC = mlContext.Model.CreatePredictionEngine<TrackerData, PredictionC>(modelC);

            var lookup = new HIKBendLookup();

            Vector3 LookupFn((Vector3 handPos, Quaternion handRot) tuple)
            {
                var pos = tuple.handPos;
                var data = new TrackerData { D = pos.X, E = pos.Y, F = pos.Z };

                var xx = engineA.Predict(data).PredictedA;
                var yy = engineB.Predict(data).PredictedB;
                var zz = engineC.Predict(data).PredictedC;

                return new Vector3(xx, yy, zz);
            }

            lookup.SetLookupFunction(LookupFn);
            lookup.BakeLookupTable();
            
            var vectors = lookup.ExportLookupTable();
            var vEnumerable = vectors
                .Select(v => string.Format(CultureInfo.InvariantCulture, "{0:0.000000},{1:0.000000},{2:0.000000}", v.X, v.Y, v.Z));
            
            File.WriteAllText("arm_bend_lookup_table.txt", string.Join(';', vEnumerable));
        }
    }

    private void TrainingMode(MLContext mlContext)
    {
        var data = mlContext.Data.LoadFromTextFile<TrackerData>("Data/000_tracker_data_record.csv", hasHeader: false, separatorChar: ',');
        
        var dataCount = data.GetColumn<float>(nameof(TrackerData.A)).Count();
        Console.WriteLine($"Loaded {dataCount} rows of data");

        DataOperationsCatalog.TrainTestData trainTestData = default;
        if (_mode == ExecutionMode.SplitTrainAndEvaluateWithoutSaving)
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

        if (_mode == ExecutionMode.TrainAllAndExport)
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

    private enum ExecutionMode
    {
        TrainAllAndExport,
        SplitTrainAndEvaluateWithoutSaving,
        GenerateLookupTable
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
