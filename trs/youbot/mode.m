function [modeblue]=mode(a,count)
    modeblue=zeros(size(a(:,:,1)));
    switch count
        case 1
            modeblue=(a(:,:,1)>150).*(a(:,:,2)>150).*(a(:,:,3)<100);
        case 2
            modeblue=(a(:,:,1)>200).*(a(:,:,2)<100).*(a(:,:,3)<100);
        case 3
            modeblue=(a(:,:,1)<100).*(a(:,:,2)<100).*(a(:,:,3)>150);
        case 4
            modeblue=(a(:,:,1)>190).*(a(:,:,2)<110).*(a(:,:,3)<110);
        case 5
            modeblue=(a(:,:,1)<70).*(a(:,:,2)>150).*(a(:,:,3)<70);
    end
end