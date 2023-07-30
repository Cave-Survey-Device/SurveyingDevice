#include "mag_cal.h"
#include "data_generation.h"
#include "alignment2.h"
#include "laser_cal.h"

using namespace Eigen;
#define BOOST_IOSTREAMS_NO_LIB


void generateAndSaveData()
{
    // Define error parameters - taken from JCAA paper
    Matrix3f Tm;
    Tm <<  0.462,-0.0293,-0.037,
    0.0686,0.4379,0.0303,
    0.0427,-0.0336,0.4369;

    Vector3f hm;
    hm << -0.176,0.2214,0.0398;

    Matrix3f Ta;
    Ta <<  9.77,0.0018,-0.030,
    0.0019,9.7032,-0.0011,
    -0.0087, -0.0013,9.6927;
    Ta = Ta * 0.1;

    Vector3f ha;
    ha << -0.01472,-0.0011,-0.01274;

    // Generate sensor data
    Vector3f mag_vec = {1.,0.,0.};
    Vector3f accel_vec = {0.,0.,1.};

    MatrixXf mag_true_data = generateTrueInertialAlignData(mag_vec);
    MatrixXf accel_true_data = generateTrueInertialAlignData(accel_vec);

    MatrixXf mag_samples = generateInertialAlignData(mag_true_data, Tm, hm);
    MatrixXf accel_samples = generateInertialAlignData(accel_true_data, Ta, ha);

    MatrixXf laser_samples = generateLaserAlignData();

    // Save sensor data
    writeToCSVfile("mag_generated_samples.txt", mag_samples.transpose());
    writeToCSVfile("accel_generated_samples.txt", accel_samples.transpose());
    writeToCSVfile("laser_generated_samples.txt",laser_samples.transpose());
}

void testAlignmentUsingReal()
{
    // Read in data in Nx3 format
    cout << "Reading accel_samples.txt\n";
    MatrixXf accel_samples = read_from_file("accel_samples.txt");
    cout << accel_samples;

    cout << "Reading mag_samples.txt\n";
    MatrixXf mag_samples = read_from_file("mag_samples.txt");
    cout << mag_samples;

    cout << "Reading accel_corrections.txt\n";
    MatrixXf accel_corrections = read_from_file("accel_corrections.txt");
    cout << "Reading mag_corrections.txt\n";
    MatrixXf mag_corrections = read_from_file("mag_corrections.txt");


    // Data is in Nx3 format but wants to be 3xN
    accel_samples.transposeInPlace();
    mag_samples.transposeInPlace();
    accel_corrections.transposeInPlace();
    mag_corrections.transposeInPlace();

    // Run calibration
    // Declare variables
    Matrix<float, 3, 3> M;
    Vector<float, 3> n;
    float d;
    Vector<float, 10> U;
    Matrix<float, 3, 3> R;
    Vector<float, 3> b;

    // Calculate magnetometer calibration
    U = fit_ellipsoid(mag_samples);
    std::cout << "Magnetometer U:\n" << U << "\n";
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];
    Vector<float, 12> mag_transformation = calculate_transformation(M, n, d);

    R << mag_transformation[0], mag_transformation[1], mag_transformation[2], mag_transformation[3], mag_transformation[4], mag_transformation[5], mag_transformation[6], mag_transformation[7], mag_transformation[8];
    b << mag_transformation[9], mag_transformation[10], mag_transformation[11];
    mag_corrections = R * (mag_samples.colwise() - b);

    // Calculate accelerometer calibration
    U = fit_ellipsoid(accel_samples);
    std::cout << "Accelerometer U:\n" << U << "\n";

    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];

    Vector<float, 12> accel_transformation = calculate_transformation(M, n, d);
    R << accel_transformation[0], accel_transformation[1], accel_transformation[2], accel_transformation[3], accel_transformation[4], accel_transformation[5], accel_transformation[6], accel_transformation[7], accel_transformation[8];
    b << accel_transformation[9], accel_transformation[10], accel_transformation[11];
    accel_corrections = R * (accel_samples.colwise() - b);
    cout << "Accel corrections " << accel_corrections.rows() << " x " << accel_corrections.cols() << ":\n" << accel_corrections.row(0) << "\n"  << accel_corrections.row(1) << "\n"  << accel_corrections.row(2) << "\n";

    // Run alignment
    std::cout << "Beginning alignment...\n";
    Vector<float,10> X;
    X = Align2(accel_corrections,mag_corrections); //Align(accel_corrections,mag_corrections);
    std::cout << X;
}

void testAlignmentUsingFake()
{

}

MatrixXf runCaliration(MatrixXf samples)
{
    // Declare variables
    Matrix<float, 3, 3> M;
    Vector<float, 3> n;
    float d;
    Vector<float, 10> U;
    Matrix<float, 3, 3> R;
    Vector<float, 3> b;

    // Calculate calibration
    U = fit_ellipsoid(samples);
    std::cout << "Magnetometer U:\n" << U << "\n";
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];
    Vector<float, 12> transformation = calculate_transformation(M, n, d);

    R << transformation[0], transformation[1], transformation[2], transformation[3], transformation[4], transformation[5], transformation[6], transformation[7], transformation[8];
    b << transformation[9], transformation[10], transformation[11];
    MatrixXf corrections = R * (samples.colwise() - b);
    return corrections;
}

int main()
{
    Matrix<float,4,-1> laser_sample_data = generateLaserAlignData();
    Vector2f data;
    data = align_laser(laser_sample_data);
    cout << "Laser alignment data: " << RAD_TO_DEG*data(0) << " " << RAD_TO_DEG*data(1);
    writeToCSVfile("laser_sample_data.txt",laser_sample_data);
    return 0;
}
//
//int main() {
//
//    // Define error parameters - taken from JCAA paper
//    Matrix3f Tm;
//    Tm <<  0.462,-0.0293,-0.037,
//    0.0686,0.4379,0.0303,
//    0.0427,-0.0336,0.4369;
//
//    Vector3f hm;
//    hm << -0.176,0.2214,0.0398;
//
//    Matrix3f Ta;
////    Ta << 0, 0, 0;
//    Ta <<  9.77,0.0018,-0.030,
//    0.0019,9.7032,-0.0011,
//    -0.0087, -0.0013,9.6927;
//    Ta = Ta * 0.1;
//
//    Vector3f ha;
//    ha << -0.01472,-0.0011,-0.01274;
//
//    // Generate sensor data
//    Vector3f mag_vec = {1.,0.,0.};
//    Vector3f accel_vec = {0.,0.,1.};
//
//    MatrixXf mag_true_data = generateTrueInertialAlignData(mag_vec);
//    MatrixXf accel_true_data = generateTrueInertialAlignData(accel_vec);
//
//    // std::cout << mag_true_data;
//
//    MatrixXf mag_samples = generateInertialAlignData(mag_true_data, Tm, hm);
//    MatrixXf accel_samples = generateInertialAlignData(accel_true_data, Ta, ha);
//
//    // Save sensor data
//    writeToCSVfile("mag_samples.txt", mag_samples.transpose());
//    writeToCSVfile("accel_samples.txt", accel_samples.transpose());
//
////    Matrix<float,3,180> mag_samples;
////    mag_samples << -0.262324270000000,-0.261434440000000,-0.262027680000000,-0.262146320000000,-0.261671720000000,-0.261968310000000,-0.261375100000000,-0.262383610000000,-0.260900530000000,-0.262086960000000,-0.261849670000000,-0.261315820000000,-0.262146290000000,-0.261375100000000,-0.262264910000000,-0.0831108800000000,-0.0826956200000000,-0.0826363000000000,-0.0825176500000000,-0.0848312200000000,-0.0826956200000000,-0.0818650900000000,-0.0828142600000000,-0.0828735800000000,-0.0834074800000000,-0.0828735800000000,-0.0828142700000000,-0.0825769800000000,-0.0812718700000000,-0.0834668100000000,-0.187103290000000,-0.188111770000000,-0.187874480000000,-0.189357550000000,-0.188171100000000,-0.188052450000000,-0.188349070000000,-0.186747340000000,-0.187577870000000,-0.187459200000000,-0.189060930000000,-0.188467710000000,-0.187993120000000,-0.189060930000000,-0.187221930000000,0.313934860000000,0.311917930000000,0.312392470000000,0.312689100000000,0.312273830000000,0.312155190000000,0.313697580000000,0.312155220000000,0.312748430000000,0.312689130000000,0.312926410000000,0.312689100000000,0.311977240000000,0.312570480000000,0.312985720000000,0.287061750000000,0.285756680000000,0.286112610000000,0.288129570000000,0.287773640000000,0.287477050000000,0.288900760000000,0.287714300000000,0.288366850000000,0.289197390000000,0.287061750000000,0.288722780000000,0.287714300000000,0.288960100000000,0.287417710000000,-0.269680260000000,-0.269324330000000,-0.270570100000000,-0.270273480000000,-0.269205660000000,-0.271104010000000,-0.270332780000000,-0.270510760000000,-0.270926060000000,-0.269680260000000,-0.270154830000000,-0.270332810000000,-0.269265000000000,-0.270510760000000,-0.269917550000000,0.575369240000000,0.573530320000000,0.573767540000000,0.574420150000000,0.574538770000000,0.573708240000000,0.573886220000000,0.573648930000000,0.574479460000000,0.573115050000000,0.574242170000000,0.572759030000000,0.572759090000000,0.573352280000000,0.572699840000000,0.550691130000000,0.549682680000000,0.549386020000000,0.549267410000000,0.549919900000000,0.549860600000000,0.548614860000000,0.550157190000000,0.550513150000000,0.549623310000000,0.550572510000000,0.550631820000000,0.548614860000000,0.551462290000000,0.550216560000000,-0.633268120000000,-0.631132480000000,-0.632437590000000,-0.631429080000000,-0.631666360000000,-0.630657850000000,-0.631488440000000,-0.630123970000000,-0.631725670000000,-0.630776520000000,-0.631429080000000,-0.631369770000000,-0.631725670000000,-0.632318910000000,-0.630717220000000,-0.621284960000000,-0.621106980000000,-0.620276450000000,-0.621759530000000,-0.620869700000000,-0.619623900000000,-0.619445920000000,-0.621106980000000,-0.621581550000000,-0.620395120000000,-0.621640860000000,-0.620751020000000,-0.620276450000000,-0.620217080000000,-0.620513740000000,0.940499600000000,0.940084280000000,0.940262200000000,0.939431790000000,0.938838480000000,0.935397800000000,0.939787690000000,0.938482580000000,0.938245300000000,0.938067320000000,0.937770720000000,0.938660500000000,0.937592690000000,0.939075770000000,0.938957210000000,0.959364120000000,0.960254010000000,0.960728530000000,0.960075970000000,0.960491300000000,0.960491300000000,0.960135280000000,0.960194590000000,0.959957300000000,0.961796400000000,0.961084490000000,0.960906510000000,0.959720020000000,0.959898110000000,0.960847200000000,-0.256629290000000,-0.256036100000000,-0.256095380000000,-0.256569980000000,-0.255976740000000,-0.258349660000000,-0.255858120000000,-0.257103860000000,-0.256273390000000,-0.255798790000000,-0.256688620000000,-0.256273330000000,-0.257163230000000,-0.256392000000000,-0.257281840000000,0.384825290000000,0.385655820000000,0.383994760000000,0.386071120000000,0.385833800000000,0.384291410000000,0.385181250000000,0.384766010000000,0.385952440000000,0.385181250000000,0.385655820000000,0.385893110000000,0.383698170000000,0.385715130000000,0.385893110000000,-0.524885830000000,-0.525657000000000,-0.525063810000000,-0.523936690000000,-0.525775670000000,-0.524589240000000,-0.526546840000000,-0.523343440000000,-0.525775610000000,-0.525123060000000,-0.524174030000000,-0.525360470000000,-0.524173970000000,-0.525834920000000,-0.525419710000000,-0.555852170000000,-0.555792870000000,-0.554962340000000,-0.555674200000000,-0.555674200000000,-0.553775910000000,-0.556682710000000,-0.555852170000000,-0.554784420000000,-0.554843660000000,-0.556801320000000,-0.554250420000000,-0.557216580000000,-0.554428460000000,-0.555852170000000,0.698522930000000,0.697455050000000,0.696505900000000,0.699056800000000,0.695497390000000,0.697573780000000,0.697158460000000,0.696683880000000,0.696980480000000,0.696980540000000,0.695853350000000,0.697988990000000,0.697158460000000,0.697336380000000,0.697988990000000,0.796523690000000,0.797354280000000,0.795040730000000,0.798006770000000,0.795989810000000,0.796939020000000,0.796405080000000,0.797413530000000,0.795871140000000,0.796938960000000,0.796583060000000,0.795455930000000,0.797116880000000,0.796820340000000,0.794862750000000,-0.866820930000000,-0.865693750000000,-0.865397160000000,-0.867236140000000,-0.867295440000000,-0.866939540000000,-0.867176890000000,-0.866642950000000,-0.868007360000000,-0.866049770000000,-0.866939540000000,-0.868244650000000,-0.865219120000000,-0.867176830000000,-0.866524340000000,-0.854363260000000,-0.852227570000000,-0.853354750000000,-0.853592040000000,-0.851575020000000,-0.853829260000000,-0.851990340000000,-0.852346240000000,-0.852286930000000,-0.853117470000000,-0.853176770000000,-0.853354750000000,-0.853117470000000,-0.854600550000000,-0.853295450000000,0.470131100000000,0.467876850000000,0.470190440000000,0.468351420000000,0.468885330000000,0.469003980000000,0.467580260000000,0.467402280000000,0.468470100000000,0.467817540000000,0.468054830000000,0.470190440000000,0.468707380000000,0.469181950000000,0.468232780000000,0.0627632500000000,0.0653141100000000,0.0646022400000000,0.0648988600000000,0.0637717300000000,0.0638310500000000,0.0642463100000000,0.0618140800000000,0.0654327600000000,0.0643649500000000,0.0650175100000000,0.0627632500000000,0.0632378300000000,0.0651954700000000,0.0630005400000000,0.107551770000000,0.108085670000000,0.107729730000000,0.108738220000000,0.107373790000000,0.110695850000000,0.109034820000000,0.109153470000000,0.108500920000000,0.109806020000000,0.112119590000000,0.109390760000000,0.110161950000000,0.109984000000000,0.108856860000000,-0.258646280000000,-0.258349630000000,-0.259951350000000,-0.261197150000000,-0.260663240000000,-0.258883540000000,-0.258408990000000,-0.259654760000000,-0.258764920000000,-0.258824260000000,-0.259773400000000,-0.261493770000000,-0.258408960000000,-0.258230980000000,-0.259417470000000,0.758616570000000,0.758675930000000,0.756718280000000,0.758438710000000,0.759269120000000,0.759862360000000,0.758498010000000,0.758498010000000,0.759091200000000,0.757074240000000,0.759031890000000,0.758794490000000,0.759565710000000,0.758675930000000,0.759565830000000,0.759209870000000,0.759625080000000,0.758853970000000,0.758141930000000,0.758497950000000,0.759625080000000,0.759447100000000,0.758735300000000,0.758260670000000,0.758379340000000,0.758201300000000,0.759625020000000,0.759328480000000,0.758972470000000,0.756836950000000,-0.814320450000000,-0.813786630000000,-0.815210340000000,-0.815388320000000,-0.814201830000000,-0.813193320000000,-0.812718750000000,-0.813489910000000,-0.813727260000000,-0.814913750000000,-0.814439120000000,-0.814676460000000,-0.814617160000000,-0.813490030000000,-0.812540830000000,-0.838583470000000,-0.840244410000000,-0.839176650000000,-0.838108840000000,-0.840244470000000,-0.838346120000000,-0.839117350000000,-0.838820760000000,-0.838464740000000,-0.839057920000000,-0.838108840000000,-0.838939310000000,-0.837752940000000,-0.838642720000000,-0.838168140000000,0.495283900000000,0.493919460000000,0.495580490000000,0.496055070000000,0.495402510000000,0.495699110000000,0.496351660000000,0.494156750000000,0.496292350000000,0.494156690000000,0.496173680000000,0.494216050000000,0.493978800000000,0.495580490000000,0.493622870000000,0.118882370000000,0.117399300000000,0.114670470000000,0.117399300000000,0.115856920000000,0.113958600000000,0.117577270000000,0.115085720000000,0.114373850000000,0.117517950000000,0.114611150000000,0.115797600000000,0.116806070000000,0.115619640000000,0.114670470000000,0.0800854300000000,0.0797888200000000,0.0786616900000000,0.0813905100000000,0.0800261100000000,0.0829922300000000,0.0817464500000000,0.0819244200000000,0.0821617100000000,0.0831108800000000,0.0821617100000000,0.0826956200000000,0.0806193300000000,0.0812125600000000,0.0819837500000000,-0.274366770000000,-0.274248060000000,-0.274782030000000,-0.273714210000000,-0.275137960000000,-0.275256570000000,-0.274248060000000,-0.274900650000000,-0.276027770000000,-0.276680320000000,-0.272409110000000,-0.275493860000000,-0.273595540000000,-0.272290470000000,-0.273180310000000,-0.246307180000000,-0.244883450000000,-0.246366500000000,-0.251408930000000,-0.247374980000000,-0.249629230000000,-0.248680070000000,-0.248027530000000,-0.246425820000000,-0.248858050000000,-0.247730930000000,-0.246663090000000,-0.249985170000000,-0.250163140000000,-0.250044490000000,0.465681940000000,0.466215790000000,0.464376840000000,0.464732740000000,0.463842930000000,0.465681880000000,0.465266700000000,0.466749760000000,0.465325980000000,0.465444620000000,0.463783590000000,0.463605640000000,0.466631080000000,0.466512440000000,0.465800580000000,-0.535326600000000,-0.534436760000000,-0.533724900000000,-0.535623190000000,-0.535563890000000,-0.533902820000000,-0.535445210000000,-0.536275740000000,-0.535919850000000,-0.534851970000000,-0.537402810000000,-0.537699460000000,-0.537046910000000,-0.536691010000000,-0.536097820000000,-0.471732800000000,-0.469656530000000,-0.471732800000000,-0.470427750000000,-0.468410790000000,-0.469122650000000,-0.470427690000000,-0.470842990000000,-0.468114140000000,-0.469893840000000,-0.468944700000000,-0.468114140000000,-0.469063340000000,-0.469597190000000,-0.471732800000000;
////    Matrix<float,3,180> accel_samples;
////    accel_samples << 0.0936666600000000,0.0935555500000000,0.0934444500000000,0.0932222200000000,0.0938888900000000,0.0949629600000000,0.0931481500000000,0.0925555600000000,0.0933333300000000,0.0936296400000000,0.0938888900000000,0.0942592600000000,0.0933333300000000,0.0935185200000000,0.0929259300000000,0.00255556000000000,0.00185185000000000,0.00188889000000000,0.00185185000000000,0.00218519000000000,0.00185185000000000,0.00192593000000000,0.00277778000000000,0.00222222000000000,0.00181481000000000,0.00240741000000000,0.00159259000000000,0.00203704000000000,0.00185185000000000,0.00129630000000000,0.0214814800000000,0.0220740700000000,0.0222222200000000,0.0223703700000000,0.0229259300000000,0.0228518500000000,0.0242962900000000,0.0220000000000000,0.0227037000000000,0.0219629600000000,0.0217407400000000,0.0226666700000000,0.0219259300000000,0.0229629600000000,0.0229629600000000,-0.152296290000000,-0.153407410000000,-0.151703720000000,-0.151555570000000,-0.152222220000000,-0.152111110000000,-0.152407410000000,-0.153222230000000,-0.152703720000000,-0.151481480000000,-0.152666670000000,-0.152407410000000,-0.152851850000000,-0.151629640000000,-0.154222220000000,0.0255185200000000,0.0269259300000000,0.0253703700000000,0.0251851900000000,0.0250000000000000,0.0250000000000000,0.0260740800000000,0.0255555500000000,0.0251851900000000,0.0250000000000000,0.0249259300000000,0.0251851900000000,0.0250370400000000,0.0250000000000000,0.0248148100000000,0.0398148100000000,0.0379629700000000,0.0401851900000000,0.0391111200000000,0.0398518500000000,0.0389629600000000,0.0390370400000000,0.0381851900000000,0.0403703700000000,0.0395925900000000,0.0396296300000000,0.0388888900000000,0.0398148100000000,0.0409259300000000,0.0382962900000000,-0.0492592600000000,-0.0460740700000000,-0.0446666700000000,-0.0472222200000000,-0.0474074100000000,-0.0457777800000000,-0.0447407400000000,-0.0476666700000000,-0.0472222200000000,-0.0475555600000000,-0.0453333300000000,-0.0461111100000000,-0.0462222200000000,-0.0449629600000000,-0.0505925900000000,-0.0493333300000000,-0.0505925900000000,-0.0508148100000000,-0.0505555600000000,-0.0500740800000000,-0.0501851800000000,-0.0516666700000000,-0.0503333300000000,-0.0510370300000000,-0.0512592600000000,-0.0506666700000000,-0.0502963000000000,-0.0492592600000000,-0.0496296300000000,-0.0504814900000000,0.988518540000000,0.989148140000000,0.987222190000000,0.988629640000000,0.987962900000000,0.988074120000000,0.988074120000000,0.986629660000000,0.988740740000000,0.988962950000000,0.986148180000000,0.988555550000000,0.987740700000000,0.985851880000000,0.985666630000000,0.985000010000000,0.984629630000000,0.984629630000000,0.984925930000000,0.984666650000000,0.984518530000000,0.985333320000000,0.985259230000000,0.985000010000000,0.985000010000000,0.984740730000000,0.984925930000000,0.984703720000000,0.984703720000000,0.984888910000000,-1.00140738000000,-1.00055552000000,-1.00166667000000,-1.00099993000000,-1.00185180000000,-1.00122225000000,-1.00137031000000,-1.00074077000000,-1.00148153000000,-1.00114799000000,-1.00022209000000,-1.00129628000000,-1.00074077000000,-1.00074077000000,-1.00092590000000,-1.00348151000000,-1.00362957000000,-1.00433338000000,-1.00392592000000,-1.00414813000000,-1.00388885000000,-1.00348151000000,-1.00307405000000,-1.00296295000000,-1.00370371000000,-1.00340724000000,-1.00396287000000,-1.00385189000000,-1.00351846000000,-1.00440753000000,0.0246666700000000,0.0224814800000000,0.0236296300000000,0.0245185200000000,0.0235185200000000,0.0244074100000000,0.0231481500000000,0.0240740700000000,0.0232963000000000,0.0233703700000000,0.0238888900000000,0.0238148200000000,0.0250000000000000,0.0231481500000000,0.0235925900000000,0.0289629600000000,0.0285185200000000,0.0290370400000000,0.0294444400000000,0.0291111100000000,0.0300370400000000,0.0295185200000000,0.0290740800000000,0.0291111100000000,0.0288518500000000,0.0290370400000000,0.0298148200000000,0.0290370400000000,0.0285185200000000,0.0290740800000000,0.106074070000000,0.105925930000000,0.105555560000000,0.105333330000000,0.105407400000000,0.106444440000000,0.106518520000000,0.106888890000000,0.105111110000000,0.105296300000000,0.105481480000000,0.105740740000000,0.105185190000000,0.105555560000000,0.105555560000000,0.0240740700000000,0.0251111100000000,0.0265185200000000,0.0260370400000000,0.0257037000000000,0.0263703700000000,0.0253703700000000,0.0250370400000000,0.0242222200000000,0.0243703700000000,0.0261111100000000,0.0242592600000000,0.0256296300000000,0.0268888900000000,0.0236666600000000,-0.943740730000000,-0.945296290000000,-0.943703770000000,-0.943888840000000,-0.944074030000000,-0.944037080000000,-0.944000070000000,-0.944000070000000,-0.944000070000000,-0.944037080000000,-0.944074030000000,-0.943888840000000,-0.943703770000000,-0.943888840000000,-0.944074030000000,-0.986518500000000,-0.988666710000000,-0.987555620000000,-0.988222300000000,-0.987481470000000,-0.989259240000000,-0.988111140000000,-0.989407420000000,-0.987185180000000,-0.989444430000000,-0.988333340000000,-0.989296260000000,-0.987777830000000,-0.987296280000000,-0.989407360000000,1.00048149000000,0.999555590000000,0.995925900000000,0.997629640000000,0.998148140000000,0.996814850000000,0.997999970000000,0.998740790000000,0.999481500000000,0.996259330000000,0.997814770000000,0.998370350000000,0.995407400000000,0.996629600000000,0.992148100000000,0.993925930000000,0.992074130000000,0.990925910000000,0.991222200000000,0.992296340000000,0.992629650000000,0.992740750000000,0.990703700000000,0.990518570000000,0.993111010000000,0.994333390000000,0.991259280000000,0.985185270000000,0.991740700000000,0.991481480000000,-0.121777770000000,-0.123185180000000,-0.121481480000000,-0.120851850000000,-0.119148150000000,-0.119777780000000,-0.120222230000000,-0.118074070000000,-0.123111110000000,-0.122518520000000,-0.118555550000000,-0.121259260000000,-0.119925930000000,-0.117703710000000,-0.117814820000000,0.0790740800000000,0.0791481500000000,0.0791851900000000,0.0790370400000000,0.0790740800000000,0.0791851900000000,0.0788888900000000,0.0790740800000000,0.0790370400000000,0.0793703700000000,0.0790740800000000,0.0790740800000000,0.0803703700000000,0.0794444500000000,0.0791481500000000,-0.0178148200000000,-0.0156296300000000,-0.0187777800000000,-0.0175925900000000,-0.0170000000000000,-0.0178518500000000,-0.0170000000000000,-0.0199259300000000,-0.0196296300000000,-0.0189259300000000,-0.0160740700000000,-0.0175925900000000,-0.0198518500000000,-0.0192963000000000,-0.0200370400000000,0.0450740800000000,0.0439629600000000,0.0430000000000000,0.0442962900000000,0.0427407400000000,0.0432592600000000,0.0436296300000000,0.0467777800000000,0.0487037000000000,0.0445925900000000,0.0463703700000000,0.0443333300000000,0.0453703700000000,0.0451111100000000,0.0418888900000000,0.996851800000000,0.996185180000000,0.995666680000000,0.997185230000000,0.996481540000000,0.997037050000000,0.996444400000000,0.997777760000000,0.996074080000000,0.996222200000000,0.995999990000000,0.996851920000000,0.996444400000000,0.995740710000000,0.997222240000000,1.00307393000000,0.998740790000000,1.00185180000000,1.00029624000000,1.00125921000000,0.999962930000000,0.999296310000000,0.999740720000000,1.00070381000000,1.00092590000000,1.00118518000000,1.00051856000000,1.00114799000000,0.998000030000000,1.00177789000000,-0.994925860000000,-0.992703740000000,-0.991926010000000,-0.991407390000000,-0.991814790000000,-0.992407440000000,-0.993148150000000,-0.995370390000000,-0.995629670000000,-0.995000000000000,-0.995000060000000,-0.993851840000000,-0.991740700000000,-0.991888880000000,-0.991592590000000,-0.985259230000000,-0.985888840000000,-0.985999940000000,-0.987814780000000,-0.985999940000000,-0.987851920000000,-0.988333340000000,-0.990444480000000,-0.987074080000000,-0.988111140000000,-0.988036990000000,-0.987629710000000,-0.988259200000000,-0.990444480000000,-0.987037060000000,0.319259260000000,0.319592600000000,0.319370390000000,0.319222240000000,0.319444450000000,0.319629640000000,0.319259260000000,0.319407400000000,0.319629640000000,0.319444450000000,0.319629640000000,0.319629640000000,0.319444450000000,0.319814800000000,0.319814800000000,0.129962970000000,0.120629630000000,0.122740750000000,0.121148150000000,0.124000000000000,0.116740750000000,0.122222220000000,0.117444440000000,0.125851840000000,0.115370370000000,0.121222220000000,0.116814810000000,0.123037040000000,0.124074080000000,0.116851850000000,-0.0730370400000000,-0.0744074100000000,-0.0710370400000000,-0.0695555600000000,-0.0700370400000000,-0.0669259300000000,-0.0691111100000000,-0.0695185200000000,-0.0709259300000000,-0.0700370400000000,-0.0685555500000000,-0.0686666700000000,-0.0705925900000000,-0.0643333300000000,-0.0633703700000000,-0.132037040000000,-0.131481480000000,-0.131518510000000,-0.132296290000000,-0.132111120000000,-0.130740750000000,-0.129370360000000,-0.130370360000000,-0.132999990000000,-0.130703720000000,-0.132000000000000,-0.130814820000000,-0.126481490000000,-0.127925920000000,-0.130074070000000,-0.0267037000000000,-0.0272592600000000,-0.0278518500000000,-0.0249629600000000,-0.0242222200000000,-0.0268518500000000,-0.0285185200000000,-0.0267037100000000,-0.0311851900000000,-0.0297777800000000,-0.0332592600000000,-0.0270740700000000,-0.0288518500000000,-0.0310370400000000,-0.0264444400000000,0.128407400000000,0.129925920000000,0.129666660000000,0.129259260000000,0.128999990000000,0.129333330000000,0.131740730000000,0.130888890000000,0.130000000000000,0.130555560000000,0.131407410000000,0.130777790000000,0.130333330000000,0.130148140000000,0.129814820000000,-0.0836666700000000,-0.0807777800000000,-0.0814074100000000,-0.0832592700000000,-0.0847407400000000,-0.0825925900000000,-0.0833333400000000,-0.0824814800000000,-0.0844814800000000,-0.0843333300000000,-0.0832963000000000,-0.0839629600000000,-0.0842592600000000,-0.0836296300000000,-0.0838148100000000,-0.0197037000000000,-0.0194814800000000,-0.0233703700000000,-0.0222222200000000,-0.0214814800000000,-0.0206666700000000,-0.0192592600000000,-0.0179629600000000,-0.0187407400000000,-0.0199629600000000,-0.0177407400000000,-0.0222592600000000,-0.0182963000000000,-0.0189629600000000,-0.0228518500000000;
////
//
//    // Declare variables
//    Matrix<float, 3, 3> M;
//    Vector<float, 3> n;
//    float d;
//    Vector<float, 10> U;
//    Matrix<float, 3, 3> R;
//    Vector<float, 3> b;
//
//    // Calculate magnetometer calibration
//    U = fit_ellipsoid(mag_samples);
//    std::cout << "Magnetometer U:\n" << U << "\n";
//    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
//    n << U[6], U[7], U[8];
//    d = U[9];
//    Vector<float, 12> mag_transformation = calculate_transformation(M, n, d);
//
//    R << mag_transformation[0], mag_transformation[1], mag_transformation[2], mag_transformation[3], mag_transformation[4], mag_transformation[5], mag_transformation[6], mag_transformation[7], mag_transformation[8];
//    b << mag_transformation[9], mag_transformation[10], mag_transformation[11];
//    MatrixXf mag_corrections = R * (mag_samples.colwise() - b);
//
//    // Calculate accelerometer calibration
//    U = fit_ellipsoid(accel_samples);
//    std::cout << "Accelerometer U:\n" << U << "\n";
//
//    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
//    n << U[6], U[7], U[8];
//    d = U[9];
//
//    Vector<float, 12> accel_transformation = calculate_transformation(M, n, d);
//    R << accel_transformation[0], accel_transformation[1], accel_transformation[2], accel_transformation[3], accel_transformation[4], accel_transformation[5], accel_transformation[6], accel_transformation[7], accel_transformation[8];
//    b << accel_transformation[9], accel_transformation[10], accel_transformation[11];
//    MatrixXf accel_corrections = R * (accel_samples.colwise() - b);
//
//    std::cout << "Beginning alignment...\n";
//    Vector<float,10> X;
//    X = Align2(accel_corrections,mag_corrections); //Align(accel_corrections,mag_corrections);
//    std::cout << X;
//
//
//    writeToCSVfile("accel_corrections.txt", accel_corrections.transpose());
//    writeToCSVfile("mag_corrections.txt", mag_corrections.transpose());
//
////    // Run joint accelerometer and magnetometer calibration and alignment
////    StructUnknowns correction_parameters = jcaa(mag_samples, accel_samples);
////    MatrixXf mag_corrections = correction_parameters.R * correction_parameters.Hm * (mag_samples.colwise() - correction_parameters.Hm.inverse()*correction_parameters.vm);
////    MatrixXf accel_corrections = correction_parameters.Ha * (accel_samples.colwise() - correction_parameters.Ha.inverse()*correction_parameters.va);
//
////    writeToCSVfile("mag_partial_corrections.txt", mag_partial_corrections.transpose());
////    writeToCSVfile("accel_corrections.txt", accel_corrections.transpose());
////    writeToCSVfile("mag_corrections.txt", mag_corrections.transpose());
////
////
////    // MAG GNU PLOTTING
////    Gnuplot gp;
////    auto plots = gp.splotGroup();
////    plots.add_plot1d_colmajor(mag_true_data, "with points title 'True Vector'");
////    plots.add_plot1d_colmajor(mag_samples, "with points title 'Samples'");
////    plots.add_plot1d_colmajor(mag_corrections, "with points title 'Corrected data'");
////    gp << plots;
////
////#ifdef _WIN32
////    // For Windows, prompt for a keystroke before the Gnuplot object goes out of scope so that
////    // the gnuplot window doesn't get closed.
////    std::cout << "Press enter to exit." << std::endl;
////    std::cin.get();
////#endif
//    return 0;
//}
