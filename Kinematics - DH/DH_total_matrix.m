function M03=DH_total_matrix(DHpar)

    M01=DHmat(DHpar(1,:));
    M12=DHmat(DHpar(2,:));
    M23=DHmat(DHpar(3,:));

    M03=M01*M12*M23;