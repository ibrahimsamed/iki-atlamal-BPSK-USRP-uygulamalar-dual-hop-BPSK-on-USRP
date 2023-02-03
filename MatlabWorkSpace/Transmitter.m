% EHB 481 - Temel Haberleşme Sistemleri Tasarımı ve Uygulamaları (CRN :12070)
% Grup A1 :  İki-atlamalı (dual-hop) işbirlikli iletimin BPSK kullanarak SDR ile gerçeklenmesi
% Source node - SDR transmitter code
%% SDR Transmitter objesi 
tx = comm.SDRuTransmitter(...
              'Platform','B200', ...
              'SerialNum','31FD9C8', ...
              'ChannelMapping',1, ...
              'CenterFrequency',1200e6, ...
              'Gain',60,...
              'MasterClockRate', 56e6);
disp(tx)

%% Seed parametresi ile random semboller üretme
M = 2; % BPSK modulasyon derecesi
m = ceil(log2(M));
N = 1000;

seed = 10;
rng(seed);

data = randi([0 1],m,N);
%% M-PSK modulasyon 

s = exp( 1i * 2.0 * pi * (0:1:M) / M ); % Sembol listesi

power_list = 2.^(0 :1:m-1 );
d = ( power_list * data ); % Binary gösterimden decimal gösterime -> Örneğin 01:1 , 11:3
psk_modulated = s (d+1); % Sembol eşleştirmesi - Complex module edilmiş işaretler 

%% Alıcı tarafında frame senkronizasyonu yapmak için frame header 

% Zadoff Chu sequence
frame_header_length = 63
zadoff_chu = exp(-1i*pi*(0:frame_header_length-1).*((1:frame_header_length))/frame_header_length)

% Alıcı tarafta faz kaymasını düzeltmek için eklenen plot bitler
pltr = [1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1]

x_withzadoff = [zadoff_chu pltr  psk_modulated  pltr]

%% Oversampling
os = 8;
x_P = zeros(N * os , 1);
x_P(1:os:length(x_withzadoff)*os) = x_withzadoff; 

%%  Rcos kullanarak Pulse shaping
% rcos parameters 
beta = 0.55;
span = 10;
sps = 8;
g_T = rcosdesign(beta,span,sps,'sqrt');

x = conv( x_P , g_T );

%% Frameler arasında sessiz bölgeler
x_withzeros = [zeros(80,1);  x ; zeros(80,1)]
t = (0:1:length(x_withzeros)-1)'

%% İşareti temel banddan daha yukarıya çıkarma işlemi
highFrequencySignal = x_withzeros .* cos(2*pi*0.9*t);

%% Elde edilen işaret sürekli gönderilmesi
while true
    tx(highFrequencySignal);
    disp("Transmitted")
end
