% EHB 481 - Temel Haberleşme Sistemleri Tasarımı ve Uygulamaları (CRN :12070)
% Grup A1 :  İki-atlamalı (dual-hop) işbirlikli iletimin BPSK kullanarak SDR ile gerçeklenmesi
% Relay node
%% SDR Receiver objesi
clear

rx = comm.SDRuReceiver( ...
    'Platform','B200' ,...  
    'SerialNum','31FD9D5', ...
    'CenterFrequency',1200e6,'SamplesPerFrame',25000, 'MasterClockRate', 56e6 ,...
    'Gain',55);

%% SamplesPerFrame kadar bilgi işareti alma işlemi

X = rx();
rxSig = double(X); % Alınan işaret ile aşağıdaki algoritmaların 
% çalışabilmesi için double data tipine çevrildi.

%% Down conversion işlemi   
t = (0:1:length(rxSig)-1)';
fromHighFreq = transpose(rxSig .* cos(2*pi*0.9*t));

%% Filtre eşleme bloğu
% rcos parameters
beta = 0.55;
span = 10;
sps = 8;
g_R = rcosdesign(beta,span,sps,"sqrt");

% Göndericide rcos kullanıldığı için Rcos filtresi ile eşleme yapıldı.
afterRaisedCosine = conv( fromHighFreq , g_R );

fs = 56e6; % Symbol rate

%% Coarse Frequency Synchronization bloğu
 
psd = fftshift(abs(fft(afterRaisedCosine.^2,2^22)));
f = linspace(-fs/2.0, fs/2.0, length(psd));
[maxfreq,max_freq_ind] = max(psd);
max_freq = f(max_freq_ind);
Ts = 1/fs; % calc sample period
t = 0: Ts : (Ts*(length(afterRaisedCosine)-1)) ;% create time vector
frequencyCorrected = afterRaisedCosine .* exp(-1i*2*pi*max_freq*t/2.0);

%% Zaman senkranizasyonu

timingRecSignal = THAL_meyr_oeder_symbol_sync(8,64,frequencyCorrected);

%% Frame senkranizasyonu
% Zadoff Chu sequence
frame_header_length = 63;
zadoff_chu = exp(-1i*pi*(0:frame_header_length-1).*((1:frame_header_length))/frame_header_length);

% Gelen işaretin zadoff chu kodları ile korelasyonunu alarak framelerin başlangıç noktaları bulundu. 
corr = xcorr(zadoff_chu,timingRecSignal(end:-1:1));
corr = corr / max(corr); % Korelasyon sonucu normalize edildi.
corr_index = find(corr > 0.5); % Korelasyonu yüksek olan noktalar bulundu.
frame_start = corr_index(1)+1;

%% Faz kaymasının düzeltilmesi

% Pilot işareti
pltr = [1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1];

% Alınan pilot işaret ile gönderilen pilot işareti arasındaki fark
% kullanılarak alınan işaretteki faz kayması düzeltildi.
received_pilot = timingRecSignal(frame_start:frame_start+33);

% Faz farkının hesaplanması ve düzeltilmesi
phase_difference = angle(sum(received_pilot*pltr'/sqrt(mean(abs(received_pilot).^2))));
phase_corrected = timingRecSignal(frame_start+34:frame_start+1033) *exp(-1i*phase_difference);

%% Alınan işaretten bitlerin elde edilmesi
decoded_bits = sign(phase_corrected); % Bpsk ile module edildiği için işaretin pozitifliğine bakılarak decode edildi.
received_bits = real(decoded_bits) < 0;

received_bits_2 = real(decoded_bits) > 0;
%% Relay node'un aldığı işaretteki bit error hatasının hesaplanması

M = 2;
m = ceil(log2(M));
N = 1000;

seed = 10;
rng(seed);
send_bits = randi([0 1],m,N);

[number,ratio] = biterr(send_bits,received_bits)

disp(ratio);
release(rx)

%% Middle hop transmitter objesi

tx = comm.SDRuTransmitter(...
              'Platform','B200', ...
              'SerialNum','31FD9D5', ...
              'ChannelMapping',1, ...
              'CenterFrequency',400e6, ...
              'Gain',55,...
              'MasterClockRate', 56e6);
disp(tx)

%% Alınan işaratin tekrar BPSK Modulasyonu yapılması

M = 2;
m = ceil(log2(M));
N = length(received_bits);

data = received_bits;

%% Relay transmitter psk modulasyonu 
s = exp( 1i * 2.0 * pi * (0:1:M) / M );
power_list = 2.^(0 :1:m-1 );
d = ( power_list * data  ); 
a = s (d+1);

%% Relay transmitter frame header ve pilot işareti eklenmesi
% Zadoff Chu sequence
frame_header_length = 63;
zadoff_chu = exp(-1i*pi*(0:frame_header_length-1).*((1:frame_header_length))/frame_header_length);

pltr = [1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1];
x_with_zadoff = [zadoff_chu pltr  a  pltr];

%% Relay transmitter oversampling
os = 8;
x_P = zeros(N * os , 1);
x_P(1:os:length(x_with_zadoff)*os) = x_with_zadoff; 

%% Relay transmitter Rcos ile pulse shaping işleminin yapılması

% rcos parameters 
beta = 0.55;
span = 10;
sps = 8;
g_T = rcosdesign(beta,span,sps,'sqrt');

x = conv( x_P , g_T );

%% Relay transmitter frameler arasında sessiz bölgeler eklenmesi
x_withzeros = [zeros(80,1);  x ; zeros(80,1)];
t = (0:1:length(x_withzeros)-1)';

%% Relay transmitter İşareti temel banddan daha yukarıya çıkarma işlemi
highFrequencySignal = x_withzeros .* cos(2*pi*0.9*t);

%% Relay transmitter alınan işaretin sürekli gönderilmesi
while true
    tx(highFrequencySignal);
    disp("Transmitted")
end


